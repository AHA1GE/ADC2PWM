/*
 * ADC2PWM firmware for CH32V003 onboard ESC with DRV8311.
 *
 * Behavior:
 * 1) Output zero duty during boot window with slow LED blink.
 * 2) Require ADC <= ADC_ARM_THRESHOLD once to unlock normal control.
 * 3) After unlock, map ADC to DRV8311 duty cycle with asymmetric ramp limiting.
 * 4) Monitor DRV8311 nFAULT and SPI status for hardware faults.
 * 5) Read 3-phase current via DRV8311 CSA and display bus current on OLED.
 */

#include "drv8311.h"
#include "OLED.h"

#define ENABLE_SCREEN 1

/*================================
 * Pins and Hardware
================================*/

#define VOLTAGE_REFERENCE_MILLIVOLTS 3400ULL  // should be 3v3, but TPS63060 here actually outputs 3.4V
#define ADC_MAX_VALUE 1023

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
#define CSA_GAIN_MV_PER_A    500               // 500 mV per amp (for fixed-point math)

// Peripherals
#define ADC_PIN              PA2   // ADC0, throttle potentiometer
#define BATT_PIN             PD6   // ADC6, 4.7k/10k divider
#define LED_PIN              PD4   // active-low, open-drain
#define LED_ACTIVE_HIGH      0
#define SDA_PIN              PC1
#define SCL_PIN              PC2

/*================================
 * Constants
================================*/

// ---------------- DRV8311 PWM ----------------
#define DRV8311_PWM_PERIOD   2000   // 12-bit, ~12.5kHz with 25MHz internal osc

// ---------------- Duty Range ----------------
#define DUTY_BOOT            0      // motor stopped
#define DUTY_MIN             0      // motor stopped
#define DUTY_MAX             DRV8311_PWM_PERIOD  // full speed

// ---------------- PWM Ramp ----------------
#define RAMP_STEP_UP         4      // was RAMP_STEP_US_UP=2, scaled ×2
#define RAMP_STEP_DOWN       8      // was RAMP_STEP_US_DOWN=4, scaled ×2

// ---------------- Timing ----------------
#define BOOT_TIME_MS         2100
#define LOOP_DELAY_MS        10

// ---------------- Safety Latch ----------------
#define ADC_ARM_THRESHOLD    50

// ---------------- Battery Thresholds ----------------
#define BATTERY_THRESHOLD_1S_MIN 3000  // 3.0V (1s battery, critical low voltage)
#define BATTERY_THRESHOLD_1S_LOW 3300  // 3.3V (1s battery, warning low voltage)
#define BATTERY_THRESHOLD_1S_MAX 4500  // 4.5V (1s battery, 4.35V full charge (LiHv) + 0.15V margin)
#define BATTERY_THRESHOLD_2S_MIN 6000  // 6.0V (2s battery, critical low voltage)
#define BATTERY_THRESHOLD_2S_LOW 6500  // 6.5V (2s battery, warning low voltage)
#define BATTERY_THRESHOLD_2S_MAX 8900  // 8.9V (2s battery, 8.7V full charge (LiHv) + 0.2V margin)

/*================================
 * Global Variables and State Definitions
================================*/

static uint16_t pwmOutput = DUTY_BOOT;
static uint16_t pwmTarget = DUTY_BOOT;
static uint32_t vBattMv = 0;
static uint16_t throttleAdc = 0;
static char errorMessageBuffer[10] = { 0 };  // buffer for error message, max 9 chars + null terminator

static uint16_t phaseCurrentAdc[3] = {0, 0, 0};  // raw ADC: U, V, W
static uint16_t busCurrentMa = 0;                  // bus current in milliamps

enum BatteryState : uint8_t {
    BATTERY_STATE_NORMAL = 0,
    BATTERY_STATE_LOW,
    BATTERY_STATE_ERROR,
};
static BatteryState batteryState = BATTERY_STATE_NORMAL;
static BatteryState lastBatteryState = BATTERY_STATE_NORMAL;

static drv8311_handle_t drv8311 = NULL;

static void copyErrorMessage(const char *msg) {
    strncpy(errorMessageBuffer, msg, sizeof(errorMessageBuffer) - 1);
    errorMessageBuffer[sizeof(errorMessageBuffer) - 1] = '\0';
}

/*================================
 * Helper Functions
 *
================================*/

static bool lowBatteryCheck() {
    // 1s-2s battery, divider: gnd_4.7k_battAdc_10k_vBatt, ref = VOLTAGE_REFERENCE_MILLIVOLTS
    const uint16_t rawBattAdc = analogRead(BATT_PIN);
    vBattMv = ((uint64_t)rawBattAdc * 147ULL * VOLTAGE_REFERENCE_MILLIVOLTS) / (47ULL * ADC_MAX_VALUE);

    batteryState = BATTERY_STATE_NORMAL;

    if (vBattMv <= BATTERY_THRESHOLD_1S_MIN) {
        // too low, error
        batteryState = BATTERY_STATE_ERROR;
        if (lastBatteryState != batteryState) {
            copyErrorMessage("BAT LOW");
        }
    } else if (vBattMv <= BATTERY_THRESHOLD_1S_LOW) {
        // 1s low battery, warning
        batteryState = BATTERY_STATE_LOW;
    } else if (vBattMv <= BATTERY_THRESHOLD_1S_MAX) {
        // 1s normal, do nothing
    } else if (vBattMv <= BATTERY_THRESHOLD_2S_MIN) {
        // not 1s nor 2s, error
        batteryState = BATTERY_STATE_ERROR;
        if (lastBatteryState != batteryState) {
            copyErrorMessage("BAT ERR");
        }
    } else if (vBattMv <= BATTERY_THRESHOLD_2S_LOW) {
        // 2s low battery, warning
        batteryState = BATTERY_STATE_LOW;
    } else if (vBattMv <= BATTERY_THRESHOLD_2S_MAX) {
        // 2s normal, do nothing
    } else {
        // above 2s max voltage, error
        batteryState = BATTERY_STATE_ERROR;
        if (lastBatteryState != batteryState) {
            copyErrorMessage("BAT HIGH");
        }
    }

    if (batteryState == BATTERY_STATE_NORMAL) {
        errorMessageBuffer[0] = '\0';
    }

    lastBatteryState = batteryState;
    return (batteryState == BATTERY_STATE_LOW);
}

static uint16_t adcToDutyRaw(uint16_t adcValue) {
    return (uint16_t)map((long)adcValue, 0L, (long)ADC_MAX_VALUE, 0L, (long)DRV8311_PWM_PERIOD);
}

/*================================
 * DRV8311 Interface Helpers
 *
================================*/

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

// nSLEEP callback for DRV8311 driver
static void nsleepSetCb(uint8_t level) {
    if (level)
        GPIO_SetBits(GPIOC, GPIO_Pin_0);    // PC0 HIGH = awake
    else
        GPIO_ResetBits(GPIOC, GPIO_Pin_0);  // PC0 LOW = sleep
}

// Set all three phases to the same duty cycle using CMP_PWM mode (integer API, no float)
static void motorSetDuty(uint16_t rawDuty) {
    if (drv8311 == NULL) return;
    drv8311_set_duty_raw(drv8311, rawDuty, rawDuty, rawDuty);
}

// Check DRV8311 for faults via nFAULT pin + SPI status register. Returns true if fault detected.
// nFAULT asserted (low) = hardware fault. SPI status provides the reason for display.
static bool drv8311FaultCheck(void) {
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) != Bit_RESET) {
        return false;  // nFAULT high = no fault
    }

    // nFAULT is low — always treat as fault. Try SPI for detail, fall back to generic message.
    drv8311_dev_sts1_t status = drv8311_get_status(drv8311);
    if (status.fault) {
        if (status.ocp)      copyErrorMessage("DRV OCP!");
        else if (status.ot)  copyErrorMessage("DRV OT!");
        else if (status.uvp) copyErrorMessage("DRV UVP!");
        else                 copyErrorMessage("DRV FLT!");
    } else {
        copyErrorMessage("DRV FLT!");  // SPI didn't confirm, but nFAULT is asserted
    }
    return true;
}

// Read all 3 phase currents from DRV8311 CSA outputs.
// DRV8311 CSA measures low-side FET current via analog voltage output.
// Returns the estimated bus current in milliamps (max of the 3 phase readings).
// I_mA = maxAdc * VREF_mV * 1000 / ADC_MAX / CSA_GAIN_MV_PER_A
// All constants fold at compile time — no runtime division overhead.
static uint16_t readPhaseCurrents(void) {
    phaseCurrentAdc[0] = analogRead(ISEN_U_PIN);
    phaseCurrentAdc[1] = analogRead(ISEN_V_PIN);
    phaseCurrentAdc[2] = analogRead(ISEN_W_PIN);

    // Find max ADC reading among the 3 phases
    uint16_t maxAdc = phaseCurrentAdc[0];
    if (phaseCurrentAdc[1] > maxAdc) maxAdc = phaseCurrentAdc[1];
    if (phaseCurrentAdc[2] > maxAdc) maxAdc = phaseCurrentAdc[2];

    // I_mA = maxAdc * VREF_mV * 1000 / (ADC_MAX * CSA_GAIN_MV_PER_A)
    return (uint16_t)(((uint32_t)maxAdc * VOLTAGE_REFERENCE_MILLIVOLTS * 1000UL)
                      / ((uint32_t)ADC_MAX_VALUE * CSA_GAIN_MV_PER_A));
}

#if ENABLE_SCREEN
/*================================
 * OLED Functions
 *
================================*/

static void screenInit(void) {
    OLED_Init();
    OLED_SetBrightness(50);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 20, "Init", OLED_FONT_8);
    OLED_Update();
}
static void screenClear(void) {
    OLED_Clear();
    OLED_Update();
}
static void screenShowDisarmed(void) {
    OLED_Clear();
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 20, 20, "DISARMED", OLED_FONT_8);
    OLED_Update();
}
static void screenPrintPrepare(void) {
    OLED_Clear();
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 0,  "Bat:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 12, "Thr:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 24, "DUT:", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 36, "Amp:", OLED_FONT_8);
    OLED_Update();
}
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

    // Bus current in amps with 1 decimal (from milliamps integer)
    snprintf(buffer, sizeof(buffer), "%u.%uA",
             (unsigned int)(busCurrentMa / 1000),
             (unsigned int)((busCurrentMa % 1000) / 100));
    OLED_ClearArea(32, 36, 48, 8);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 36, buffer, OLED_FONT_8);

    OLED_Update();
}
static void screenLowBatteryWarning(void) {
    // blink low battery warning at the top-right corner
    if (millis() % 1000 < 500) {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, "!", OLED_FONT_8);
    } else {
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 72, 0, " ", OLED_FONT_8);
    }
}
static void screenShowError(const char *message, uint8_t messageLength) {
    // check length, no more than 9 characters
    if (messageLength > 9) {
        // do nothing
    } else {
        OLED_Clear();
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 16, "ERROR", OLED_FONT_8);
        OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 32, message, OLED_FONT_8);
        OLED_Update();
    }
}
#endif

/*================================
 * Arduino Setup and Loop
 *
================================*/

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
    pinMode(LED_PIN, OUTPUT);

    // SPI bus idle state
    digitalWrite(DRV8311_CS_PIN, HIGH);
    digitalWrite(DRV8311_SCK_PIN, LOW);
    digitalWrite(DRV8311_MOSI_PIN, LOW);

    // --- DRV8311 init ---
    drv8311_cfg_t cfg = {
        .pwmcnt_mode = UP,
        .sync_mode = SYNC_DISABLE,
        .spi_clk = SPI_FREQ_1M,
        .spi_sync_clks = CLOCKS_512,
        .portal = SPI,
        .csa_gain = CSA_GAIN,
        .pwm_period = DRV8311_PWM_PERIOD,
        .use_csa = 1,
        .dev_id = 0,
        .parity_check = 0,
        .spi_trans = spiTransmitCb,
        .nsleep_set = nsleepSetCb
    };
    drv8311_init(&drv8311, &cfg);

    // Set all three phases to CMP_PWM mode and enable outputs
    drv8311_phase_ctrl(drv8311, PHASE_CMP_PWM, PHASE_CMP_PWM, PHASE_CMP_PWM);
    drv8311_out_ctrl(drv8311, 1);

    // --- Boot safe pulse ---
    pwmOutput = DUTY_BOOT;
    pwmTarget = DUTY_BOOT;
    motorSetDuty(DUTY_BOOT);

#if ENABLE_SCREEN
    screenInit();
#endif

    delay(BOOT_TIME_MS);

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

void loop() {
    // --- DRV8311 fault check ---
    if (drv8311FaultCheck()) {
        // lock motor, show error, halt
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
    busCurrentMa = readPhaseCurrents();

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
