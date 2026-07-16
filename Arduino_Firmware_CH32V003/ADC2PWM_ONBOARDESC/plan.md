# Plan: Rewrite ADC2UVW.ino for DRV8311 Onboard ESC Hardware

## Context

The current `ADC2UVW.ino` (274 lines) is a simple CH32V003 firmware that:
- Outputs a servo PWM pulse on PC4 via `CH32V003_SERVO.h` to drive an external ESC
- Reads throttle ADC on PA2, battery voltage on PD6
- Blocking setup: boot pulse (2s) → show DISARMED → wait for ADC ≤ 50 → unlock
- Simple loop: battery check → ADC read → ramp → PWM write → OLED update
- Low battery = 50% throttle, no error lockout

The new hardware replaces the external ESC with an onboard **DRV8311PRRWR** 3-phase BLDC driver:
- **PC4 is now DRV8311_ISEN_V (ADC2)** — conflicts with current PWM_PIN
- DRV8311 controlled via **bit-banged SPI** (CS=PC3, SCK=PC5, MOSI=PC6, MISO=PC7)
- Control pins: nSLEEP=PC0, nFAULT=PD0
- `drv8311.h/.c` driver already exists alongside the sketch, but is not included

**Outcome:** Same simple architecture, but replace Servo PWM with DRV8311 SPI control. Keep the blocking-setup + loop pattern. Add DRV8311 fault monitoring and 3-phase current sensing with real-time amp display on OLED.

---

## Files to Modify

| File | Action |
|------|--------|
| `ADC2UVW.ino` | Rewrite (~300 lines) |
| `drv8311.c` line 29 | Fix: `"drv8311_driver.h"` → `"drv8311.h"` |

---

## Implementation Steps

### Step 1: Fix DRV8311 driver include

`drv8311.c` line 29: change `#include "drv8311_driver.h"` to `#include "drv8311.h"`

### Step 2: Replace includes

Remove:
```cpp
#include "CH32V003_SERVO.h"
```
Add:
```cpp
#include "drv8311.h"
```

### Step 3: Add DRV8311 pin definitions

Replace `#define PWM_PIN SERVO_PIN_PC4_TIM1_CH4` with:

```cpp
// DRV8311 SPI & Control
#define DRV8311_nSLEEP_PIN   PC0
#define DRV8311_nFAULT_PIN   PD0
#define DRV8311_CS_PIN       PC3
#define DRV8311_SCK_PIN      PC5
#define DRV8311_MOSI_PIN     PC6
#define DRV8311_MISO_PIN     PC7

// DRV8311 3-Phase Current Sense ADCs
#define ISEN_U_PIN           PA1   // ADC1, phase U current
#define ISEN_V_PIN           PC4   // ADC2, phase V current (was PWM_PIN)
#define ISEN_W_PIN           PD2   // ADC3, phase W current

// CSA (Current Sense Amplifier) config
// DRV8311 peak = 5A. VREF = 3.4V. 500mV/A → max measurable = 3.4/0.5 = 6.8A (covers 5A with headroom).
#define CSA_GAIN             CSA_GAIN_500MV    // 0.5V per amp
#define CSA_GAIN_V_PER_A     0.5f              // volts per amp for the chosen gain
```

All other pin definitions (ADC_PIN=PA2, BATT_PIN=PD6, SDA_PIN=PC1, SCL_PIN=PC2) stay the same.

### Step 4: Replace PWM constants with DRV8311 duty constants

| Old | New | Value | Notes |
|-----|-----|-------|-------|
| `PWM_BOOT_US 890` | `DUTY_BOOT 0` | 0 | Stopped |
| `PWM_MIN_US 1000` | `DUTY_MIN 0` | 0 | Stopped |
| `PWM_MAX_US 2000` | `DUTY_MAX DRV8311_PWM_PERIOD` | 2000 | Full speed |
| `RAMP_STEP_US_UP 2` | `RAMP_STEP_UP 4` | 4 | Scaled ×2 |
| `RAMP_STEP_US_DOWN 4` | `RAMP_STEP_DOWN 8` | 8 | Scaled ×2 |

Add:
```cpp
#define DRV8311_PWM_PERIOD   2000   // 12-bit, ~12.5kHz with 25MHz internal osc
```

Keep unchanged: `VOLTAGE_REFERENCE_MILLIVOLTS`, `ADC_MAX_VALUE`, `BOOT_TIME_MS`, `LOOP_DELAY_MS`, `ADC_ARM_THRESHOLD`, all battery thresholds.

### Step 5: Replace global variables

Remove:
```cpp
static Servo esc;
```

Add:
```cpp
static drv8311_handle_t drv8311 = NULL;
```

Add:
```cpp
static uint16_t phaseCurrentAdc[3] = {0, 0, 0};  // raw ADC: U, V, W
static float busCurrentA = 0.0f;                   // bus current in amps
```

Keep: `pwmOutput`, `pwmTarget`, `vBattMv`, `throttleAdc`, `errorMessageBuffer`, `batteryState`, `lastBatteryState`.

### Step 6: Replace `adcToPulseUs()` with `adcToDutyRaw()`

```cpp
static uint16_t adcToDutyRaw(uint16_t adcValue) {
    return (uint16_t)map((long)adcValue, 0L, (long)ADC_MAX_VALUE, 0L, (long)DRV8311_PWM_PERIOD);
}
```

### Step 7: Add DRV8311 helper functions

Insert after `adcToDutyRaw()`, before the OLED section:

```cpp
// SPI bit-bang callback for DRV8311 (CPOL=0, CPHA=0, big-endian handled by driver)
static void spiTransmitCb(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len) {
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);   // CS low

    for (uint8_t i = 0; i < send_len; i++) {
        uint8_t tx_byte = send_data[i];
        uint8_t rx_byte = 0;
        for (int8_t bit = 7; bit >= 0; bit--) {
            // MOSI
            if (tx_byte & (1 << bit))
                GPIO_SetBits(GPIOC, GPIO_Pin_6);
            else
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            // SCK rising edge
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            // MISO
            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
                rx_byte |= (1 << bit);
            // SCK falling edge
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
        }
        rec_data[i] = rx_byte;
    }

    GPIO_SetBits(GPIOC, GPIO_Pin_3);     // CS high
}

static void nsleepSetCb(uint8_t level) {
    if (level)
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // PC0 HIGH = awake
    else
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // PC0 LOW = sleep
}

static void motorSetDuty(uint16_t rawDuty) {
    if (drv8311 == NULL) return;
    float duty = (float)rawDuty / (float)DRV8311_PWM_PERIOD;
    drv8311_set_duty(drv8311, duty, duty, duty);
}

static bool drv8311FaultCheck(void) {
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == Bit_RESET) {
        drv8311_dev_sts1_t status = drv8311_get_status(drv8311);
        if (status.fault) {
            if (status.ocp)      copyErrorMessage("DRV OCP!");
            else if (status.ot)  copyErrorMessage("DRV OT!");
            else if (status.uvp) copyErrorMessage("DRV UVP!");
            else                 copyErrorMessage("DRV FLT!");
            return true;
        }
    }
    return false;
}

// Read all 3 phase currents from DRV8311 CSA outputs.
// DRV8311 CSA measures low-side FET current — output is an analog voltage
// proportional to phase current: Vout = CSA_GAIN_V_PER_A * I_phase.
// Returns the estimated bus current in amps (max of the 3 phase readings).
static float readPhaseCurrents(void) {
    phaseCurrentAdc[0] = analogRead(ISEN_U_PIN);
    phaseCurrentAdc[1] = analogRead(ISEN_V_PIN);
    phaseCurrentAdc[2] = analogRead(ISEN_W_PIN);

    // Find max ADC reading among the 3 phases
    uint16_t maxAdc = phaseCurrentAdc[0];
    if (phaseCurrentAdc[1] > maxAdc) maxAdc = phaseCurrentAdc[1];
    if (phaseCurrentAdc[2] > maxAdc) maxAdc = phaseCurrentAdc[2];

    // Convert ADC → voltage → current
    // V_adc = maxAdc * VREF / ADC_MAX_VALUE
    // I_phase = V_adc / CSA_GAIN_V_PER_A
    float voltage = (float)maxAdc * (float)VOLTAGE_REFERENCE_MILLIVOLTS / 1000.0f / (float)ADC_MAX_VALUE;
    float current = voltage / CSA_GAIN_V_PER_A;
    return current;
}
```

### Step 8: Rewrite `setup()`

```cpp
void setup() {
    // --- GPIO init ---
    pinMode(ADC_PIN, INPUT);
    pinMode(BATT_PIN, INPUT);
    pinMode(DRV8311_nSLEEP_PIN, OUTPUT);
    pinMode(DRV8311_nFAULT_PIN, INPUT);
    pinMode(DRV8311_CS_PIN, OUTPUT);
    pinMode(DRV8311_SCK_PIN, OUTPUT);
    pinMode(DRV8311_MOSI_PIN, OUTPUT);
    pinMode(DRV8311_MISO_PIN, INPUT);
    pinMode(ISEN_U_PIN, INPUT);
    pinMode(ISEN_V_PIN, INPUT);  // PC4 — was PWM output, now ISEN_V ADC
    pinMode(ISEN_W_PIN, INPUT);

    // SPI bus idle state
    digitalWrite(DRV8311_CS_PIN, HIGH);
    digitalWrite(DRV8311_SCK_PIN, LOW);
    digitalWrite(DRV8311_MOSI_PIN, LOW);

    // --- DRV8311 init ---
    drv8311_cfg_t cfg = {
        .pwmcnt_mode = UP,
        .sync_mode = SYNC_DISABLE,
        .portal = SPI,
        .pwm_period = DRV8311_PWM_PERIOD,
        .use_csa = 1,
        .csa_gain = CSA_GAIN,
        .dev_id = 0,
        .parity_check = 0,
        .spi_trans = spiTransmitCb,
        .nsleep_set = nsleepSetCb
    };
    drv8311_init(&drv8311, &cfg);
    if (drv8311 == NULL) {
        // init failed — will be caught by fault check
    }
    drv8311_phase_ctrl(drv8311, PHASE_CMP_PWM, PHASE_CMP_PWM, PHASE_CMP_PWM);
    drv8311_out_ctrl(drv8311, 1);

    // --- Boot safe pulse ---
    pwmOutput = DUTY_BOOT;
    pwmTarget = DUTY_BOOT;
    motorSetDuty(DUTY_BOOT);

#if ENABLE_SCREEN
    screenInit();
#endif

    delay(BOOT_TIME_MS);   // was 2000 hardcoded, use constant

    // --- Wait for low throttle to arm ---
    pwmOutput = DUTY_MIN;
    pwmTarget = DUTY_MIN;
    motorSetDuty(pwmOutput);
#if ENABLE_SCREEN
    screenClear();
    screenShowDisarmed();
#endif
    while (analogRead(ADC_PIN) > ADC_ARM_THRESHOLD) {
        delay(LOOP_DELAY_MS);
    }

#if ENABLE_SCREEN
    screenPrintPrepare();
#endif
}
```

### Step 9: Rewrite `loop()`

Same structure but replace `esc.writeMicroseconds()` with `motorSetDuty()`, add fault check, use `adcToDutyRaw()`:

```cpp
void loop() {
    // --- DRV8311 fault check ---
    if (drv8311FaultCheck()) {
        // lock motor, fast blink, show error, halt
        motorSetDuty(DUTY_BOOT);
        drv8311_phase_ctrl(drv8311, PHASE_OFF, PHASE_OFF, PHASE_OFF);
#if ENABLE_SCREEN
        screenShowError(errorMessageBuffer, strlen(errorMessageBuffer));
#endif
        while (1) { delay(100); }   // latch until reset
    }

    // --- Battery check ---
    bool isLowBattery = lowBatteryCheck();
    if (batteryState == BATTERY_STATE_ERROR) {
        // battery error — lock motor
        motorSetDuty(DUTY_BOOT);
#if ENABLE_SCREEN
        screenShowError(errorMessageBuffer, strlen(errorMessageBuffer));
#endif
        while (1) { delay(100); }
    }

    // --- Throttle read ---
    throttleAdc = analogRead(ADC_PIN);
    if (isLowBattery) {
        throttleAdc = throttleAdc / 2;
    }

    // --- Ramp ---
    pwmTarget = adcToDutyRaw(throttleAdc);
    if (pwmOutput < pwmTarget) {
        pwmOutput = (uint16_t)(pwmOutput + RAMP_STEP_UP);
        if (pwmOutput > pwmTarget) pwmOutput = pwmTarget;
    } else if (pwmOutput > pwmTarget) {
        pwmOutput = (uint16_t)(pwmOutput - RAMP_STEP_DOWN);
        if (pwmOutput < pwmTarget) pwmOutput = pwmTarget;
    }
    motorSetDuty(pwmOutput);

    // --- Read phase currents ---
    busCurrentA = readPhaseCurrents();

    // --- OLED ---
#if ENABLE_SCREEN
    if (isLowBattery) {
        screenLowBatteryWarning();
    } else {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, " ", OLED_FONT_8);
    }
    screenPrint();
#endif

    delay(LOOP_DELAY_MS);
}
```

### Step 10: Rewrite OLED functions for 4-line layout with current

Compact line spacing from 16px to 12px to fit 4 lines (Bat / Thr / Dut / Amp) within the 56px logical framebuffer:

- y=0: `"Bat: x.xV"`
- y=12: `"Thr: xxx"`
- y=24: `"DUT: xx%"`
- y=36: `"Amp: x.xA"`

**`screenPrintPrepare()`** — update layout:
```cpp
static void screenPrintPrepare(void) {
    OLED_Clear();
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 0,  "Bat:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 12, "Thr:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 24, "DUT:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 36, "Amp:", OLED_FONT_8);
    OLED_Update();
}
```

**`screenPrint()`** — add current display, clear each value area before redraw:
```cpp
static void screenPrint(void) {
    char buffer[16];

    // Battery voltage
    snprintf(buffer, sizeof(buffer), "%lu.%luV",
             (unsigned long)(vBattMv / 1000),
             (unsigned long)((vBattMv % 1000) / 100));
    OLED_ClearArea(32, 0, 48, 8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 0, buffer, OLED_FONT_8);

    // Throttle ADC
    snprintf(buffer, sizeof(buffer), "%u", throttleAdc);
    OLED_ClearArea(32, 12, 32, 8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 12, buffer, OLED_FONT_8);

    // Duty percentage
    snprintf(buffer, sizeof(buffer), "%u%%",
             (unsigned int)((uint32_t)pwmOutput * 100UL / DRV8311_PWM_PERIOD));
    OLED_ClearArea(32, 24, 32, 8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 24, buffer, OLED_FONT_8);

    // Bus current (1 decimal place)
    snprintf(buffer, sizeof(buffer), "%u.%uA",
             (unsigned int)busCurrentA,
             (unsigned int)(busCurrentA * 10.0f) % 10);
    OLED_ClearArea(32, 36, 48, 8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 36, buffer, OLED_FONT_8);

    OLED_Update();
}
```

**`screenLowBatteryWarning()`** — move `!` blink to y=0 (top-right of battery line):
```cpp
static void screenLowBatteryWarning(void) {
    if (millis() % 1000 < 500) {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, "!", OLED_FONT_8);
    } else {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, " ", OLED_FONT_8);
    }
}
```

And in `loop()`, clear the warning area when not low battery — use y=0 instead of y=12:
```cpp
    if (isLowBattery) {
        screenLowBatteryWarning();
    } else {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, " ", OLED_FONT_8);
    }
    screenPrint();
```

---

## Summary of Changes

| What | Old | New |
|------|-----|-----|
| Include | `CH32V003_SERVO.h` | `drv8311.h` |
| PWM pin | `SERVO_PIN_PC4_TIM1_CH4` (PC4 output) | PC4 = INPUT (ISEN_V ADC) |
| Motor control | `esc.writeMicroseconds(us)` | `motorSetDuty(raw)` via SPI |
| Duty mapping | `adcToPulseUs()` → 1000-2000us | `adcToDutyRaw()` → 0-2000 raw |
| Global | `static Servo esc` | `static drv8311_handle_t drv8311` |
| Loop additions | — | DRV8311 fault check, battery error lockout |
| OLED label | `"PWM:"` + raw us | `"DUT:"` + percentage |
| New helper functions | — | `spiTransmitCb`, `nsleepSetCb`, `motorSetDuty`, `drv8311FaultCheck`, `readPhaseCurrents` |
| Boot delay | `delay(2000)` | `delay(BOOT_TIME_MS)` (same value, use constant) |
| Current sensing | — | 3-phase CSA via DRV8311 (PA1/PC4/PD2 ADCs), bus current on OLED |
| OLED layout | 3 lines, 16px spacing | 4 lines, 12px spacing (Bat/Thr/DUT/Amp) |
| DRV8311 CSA | disabled | enabled, `CSA_GAIN_500MV` (0.5V/A, ~6.8A range) |

## Verification

1. **Compile**: Build for CH32V003, verify no errors
2. **Pin audit**: Cross-check all pin macros against `PIN.md`
3. **Logic audit**: Verify all `esc.` calls replaced, all `PWM_*_US` constants replaced
4. **Driver integration**: `drv8311_init()` config uses UP mode, SYNC_DISABLE, SPI protocol, 2000 period

---

## Implementation Notes (deviations from plan)

1. **Float elimination**: All float usage removed to fit in 16KB flash. Added `drv8311_set_duty_raw()` (integer API) to the driver. Current sensing uses fixed-point math returning `uint16_t` milliamps. `busCurrentA` → `busCurrentMa`. Removed `CSA_GAIN_V_PER_A` constant.

2. **`extern "C"` guards**: Added to `drv8311.h` to fix C/C++ linkage mismatch between the .ino (C++) and `drv8311.c` (C).

3. **Designated initializer order**: Fixed `drv8311_cfg_t` initializer field order to match struct declaration (C++ requirement). Added `.spi_clk = SPI_FREQ_1M` and `.spi_sync_clks = CLOCKS_512` (unused with SYNC_DISABLE but required for correct order).

**Build result**: Flash 13,844/16,384 (84%), RAM 1,452/2,048 (70%).
