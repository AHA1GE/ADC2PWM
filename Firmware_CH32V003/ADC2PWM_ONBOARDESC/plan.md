# Plan: Rewrite ADC2UVW.ino for DRV8311 Onboard ESC Hardware

## Context

The existing `ADC2UVW.ino` is a multi-platform (AVR/CH32/ESP32) firmware that outputs a servo PWM pulse on PC4 to drive an external ESC. The new hardware (described in `PIN.md` and the schematic) integrates a **DRV8311PRRWR** 3-phase BLDC motor driver directly on the PCB. Key changes:

- **PC4 is now an ADC input** (DRV8311 ISEN_V current sense), not a PWM output
- The DRV8311 is controlled via **bit-banged SPI** (PC3=CS, PC5=SCK, PC6=MOSI, PC7=MISO) + control pins (PC0=nSLEEP, PD0=nFAULT)
- A `drv8311.h/.c` driver library already exists in the sketch directory but is not yet integrated
- Per user request: **drop the FSM library** — replace with a simple enum-based state machine

**Outcome:** A CH32V003-only firmware that reads throttle ADC on PA2, controls the DRV8311 via SPI, monitors battery voltage on PD6, drives an LED on PD4, and displays status on the onboard OLED — all with a simple switch/case state loop.

---

## Files to Modify

| File | Action |
|------|--------|
| `Firmware_CH32V003/ADC2PWM_ONBOARDESC/ADC2UVW/ADC2UVW.ino` | **Full rewrite** (~350 lines) |
| `Firmware_CH32V003/ADC2PWM_ONBOARDESC/ADC2UVW/drv8311.c` | Fix include: `"drv8311_driver.h"` → `"drv8311.h"` (line 29) |

## Files to Add

None. The FSM library is dropped. The DRV8311 driver and OLED library are already present.

---

## Implementation Steps

### Step 1: Fix DRV8311 driver include bug

`drv8311.c` line 29: `#include "drv8311_driver.h"` → `#include "drv8311.h"`

### Step 2: Rewrite `ADC2UVW.ino` header — strip multi-platform, add DRV8311

Remove `#include "Servo.h"` and `#include "Fsm.h"`. Replace with:

```cpp
#include "drv8311.h"
#include "OLED.h"
```

Remove all `#if defined(ADC2PWM_PLATFORM_xxx)` blocks. This is CH32V003-only now.

### Step 3: Define all pin macros (single platform, no conditionals)

```cpp
// DRV8311 SPI & Control
#define DRV8311_nSLEEP_PIN   PC0
#define DRV8311_nFAULT_PIN   PD0
#define DRV8311_CS_PIN       PC3
#define DRV8311_SCK_PIN      PC5
#define DRV8311_MOSI_PIN     PC6
#define DRV8311_MISO_PIN     PC7

// DRV8311 Current Sense ADCs (wired but unused in this version)
#define ISEN_U_PIN           PA1   // ADC1
#define ISEN_V_PIN           PC4   // ADC2  (was PWM_PIN — CONFLICT RESOLVED)
#define ISEN_W_PIN           PD2   // ADC3

// Peripherals (unchanged)
#define ADC_PIN              PA2   // ADC0, throttle
#define BATT_PIN             PD6   // ADC6, battery divider
#define LED_PIN              PD4   // active-low, open-drain
#define LED_ACTIVE_HIGH      0
#define ADC_MAX_VALUE        1023
```

### Step 4: Replace PWM constants with DRV8311 duty constants

| Old (servo us) | New (DRV8311 raw) | Value |
|---|---|---|
| `PWM_BOOT_US 890` | `DUTY_BOOT 0` | Motor stopped |
| `PWM_MIN_US 1000` | `DUTY_STOPPED 0` | Motor stopped |
| `PWM_MAX_US 2000` | `DUTY_MAX DRV8311_PWM_PERIOD` | Full speed |
| `RAMP_STEP_US_UP 2` | `RAMP_STEP_UP 4` | Scaled ×2 |
| `RAMP_STEP_US_DOWN 4` | `RAMP_STEP_DOWN 8` | Scaled ×2 |

DRV8311 config:
- `DRV8311_PWM_PERIOD = 2000` (12-bit, gives ~12.5kHz at 25MHz internal osc)
- Counter mode: `UP` (edge-aligned)
- Sync mode: `SYNC_DISABLE` (internal osc only)
- Protocol: `SPI` (standard 3-wire)

Battery thresholds (BATTERY_THRESHOLD_1S_MIN, etc.) and timing constants (BOOT_TIME_MS, LOOP_DELAY_MS, ADC_ARM_THRESHOLD) **remain unchanged**.

### Step 5: Replace FSM with simple enum state machine

Replace the 5 `State_t` objects and `FSM g_fsm` class with:

```cpp
typedef enum {
    STATE_BOOT,
    STATE_DISARMED,
    STATE_ARMED,
    STATE_LOW_BATTERY,
    STATE_ERROR
} esc_state_t;

static esc_state_t g_state = STATE_BOOT;
static esc_state_t g_prevState = STATE_BOOT;  // for enter/exit detection
```

### Step 6: Replace global variables

- Remove `static Servo esc;`
- Add `static drv8311_handle_t drv8311 = NULL;`
- Keep `pwmOutput`, `pwmTarget`, `bootTimeMs`, `vBattMv`, `throttleAdc`, `errorMessageBuffer[10]`

### Step 7: Add DRV8311 helper functions

Four new static functions, placed before `setup()`:

1. **`spiTransmitCb()`** — Bit-banged SPI callback (CS=PC3, SCK=PC5, MOSI=PC6, MISO=PC7). Uses direct `GPIO_SetBits`/`GPIO_ResetBits` (matches OLED driver pattern). CPOL=0, CPHA=0.

2. **`nsleepSetCb()`** — nSLEEP control callback (PC0, HIGH=awake).

3. **`motorSetDuty(uint16_t rawDuty)`** — Sets all 3 phases to same duty via `drv8311_set_duty()` with float conversion.

4. **`drv8311FaultCheck()`** — Fast check: reads nFAULT pin (PD0); if low, reads `drv8311_get_status()` for detailed fault cause, populates `errorMessageBuffer`, returns true.

### Step 8: Replace `adcToPulseUs()` with `adcToDutyRaw()`

Maps ADC 0-1023 → raw duty compare 0-DRV8311_PWM_PERIOD using `map()`.

### Step 9: Rewrite main loop with switch/case

Replace the old `loop()` (which just called `g_fsm.run()`) with a state switch:

```cpp
void loop() {
    unsigned long nowMs = millis();

    // --- State enter detection ---
    if (g_state != g_prevState) {
        // Handle enter logic for new state
        switch (g_state) {
            case STATE_BOOT:        stateBootEnter();     break;
            case STATE_DISARMED:    stateDisarmedEnter(); break;
            case STATE_ARMED:       stateArmedEnter();    break;
            // LOW_BATTERY and ERROR have no enter action
        }
        g_prevState = g_state;
    }

    // --- State run ---
    switch (g_state) {
        case STATE_BOOT:        stateBootRun(nowMs);     break;
        case STATE_DISARMED:    stateDisarmedRun(nowMs); break;
        case STATE_ARMED:       stateArmedRun();         break;
        case STATE_LOW_BATTERY: stateLowBatteryRun();    break;
        case STATE_ERROR:       stateErrorRun();         break;
    }

    delay(LOOP_DELAY_MS);
}
```

State transitions: set `g_state` and the enter logic runs next iteration.

### Step 10: Rewrite each state function

All functions keep their existing logic but replace `esc.writeMicroseconds()` with `motorSetDuty()` calls. Key changes per state:

**`stateBootEnter()`**: Initialize DRV8311 via `drv8311_init(&cfg)`, configure all 3 phases to CMP_PWM mode, set duty to 0, enable outputs. If init fails, transition to STATE_ERROR.

**`stateBootRun()`**: Wait BOOT_TIME_MS, then transition to STATE_DISARMED. LED blink at 200ms.

**`stateDisarmedEnter()`**: LED off, duty=0, show "DISARMED" on OLED.

**`stateDisarmedRun()`**: Read throttle ADC. If <= ARM_THRESHOLD → transition to STATE_ARMED. Else LED blink at 150ms, duty=0.

**`stateArmedEnter()`**: LED on, duty=0, prepare OLED layout.

**`stateArmedRun()`**: Call `lowBatteryCheck()` + `drv8311FaultCheck()`. Map ADC→duty via `adcToDutyRaw()`. Apply ramp limiting with new RAMP_STEP_UP/DOWN values. `motorSetDuty(pwmOutput)`. Update OLED.

**`stateArmedExit()`**: LED off, set duty=0.

**`stateLowBatteryRun()`**: 50% throttle limit (adcToDutyRaw(throttleAdc/2)), ramp, `motorSetDuty()`, LED blink 300ms, OLED low battery warning.

**`stateErrorRun()`**: `drv8311_phase_ctrl(PHASE_OFF, PHASE_OFF, PHASE_OFF)` to fully disconnect motor. LED fast blink 80ms. Show error on OLED.

### Step 11: Rewrite `setup()`

1. Init all GPIOs:
   - DRV8311 SPI: CS/SCK/MOSI as OUTPUT, MISO as INPUT
   - DRV8311 control: nSLEEP as OUTPUT (LOW initially), nFAULT as INPUT
   - SPI idle: CS=HIGH, SCK=LOW, MOSI=LOW
   - **PC4 = INPUT** (was PWM output, now ISEN_V ADC)
2. Init ADC, LED, BATT pins (same as before)
3. Init OLED via `OLED_Init()`
4. Start in STATE_BOOT

### Step 12: OLED updates

- In `screenPrintPrepare()`: change label from `"PWM:"` to `"DUT:"`
- In `screenPrint()`: show duty as percentage `"%u%%"` instead of raw microseconds

---

## Verification

1. **Compile check**: Build for CH32V003 target, verify no errors
2. **Code review**: Verify every `esc.writeMicroseconds()` is replaced, every `PWM_*_US` constant is replaced, every FSM transition is replaced with state enum assignment
3. **Pin audit**: Cross-reference all pin macros against `PIN.md` to ensure no conflicts
4. **Logic audit**: Verify state transitions match original FSM behavior exactly
5. **Driver integration**: Verify `drv8311_init()` config matches DRV8311 datasheet requirements for the target PWM frequency and mode
