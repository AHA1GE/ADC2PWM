/*
 * ADC2UVW — sensorless BLDC controller for CH32V003 + DRV8311P.
 *
 * Current-assisted 6-step trapezoidal commutation.
 * No Hall sensors, no BEMF voltage sensing — uses the DRV8311's CSA
 * (current sense amplifier) on the LOW-side phase to detect when the
 * rotor approaches alignment and trigger commutation.
 *
 * Motor: ~3170 KV (13,000 RPM @ 4.1 V / 100 %), 12N14P, 1S LiPo.
 * V & W phase wires physically swapped for correct rotation.
 */

#include "drv8311.h"

/*================================================================
 * Pins
 *===============================================================*/

#define VOLTAGE_REFERENCE_MILLIVOLTS 3280ULL
#define ADC_MAX_VALUE                1023

#define DRV8311_nSLEEP_PIN  PC0
#define DRV8311_nFAULT_PIN  PD0
#define DRV8311_CS_PIN      PC3
#define DRV8311_SCK_PIN     PC5
#define DRV8311_MOSI_PIN    PC6
#define DRV8311_MISO_PIN    PC7

#define ISEN_U_PIN  PA1       // phase U CSA   (ADC1)
#define ISEN_V_PIN  PC4       // phase V CSA   (ADC2)
#define ISEN_W_PIN  PD2       // phase W CSA   (ADC3)

#define CSA_GAIN          CSA_GAIN_500MV
#define CSA_GAIN_MV_PER_A 500

#define ADC_PIN   PA2         // throttle pot   (ADC0)
#define BATT_PIN  PD6         // battery divider (ADC6)
#define LED_PIN   PD4         // active-low open-drain

/*================================================================
 * Constants
 *===============================================================*/

// DRV8311 PWM — F_PWM ≈ 10 kHz (20 MHz sysclk / 2000)
#define DRV8311_PWM_PERIOD 2000

// Commutation speed range (from motor measurement: period_us × duty_pct ≈ 11000)
// 110 µs → 100 % duty → ≈13,000 RPM      5000 µs → ≈2.2 % → floored to 15 %
#define COMM_STEP_FAST_US   110UL
#define COMM_STEP_SLOW_US  5000UL

// Current-assisted commutation
#define CURRENT_ASSIST_PCT   55     // commutate when I < 55 % of step peak
#define CURRENT_NOISE_FLOOR  20     // ADC counts — ignore readings below
#define TIMEOUT_MARGIN_PCT  250     // timeout = SLOW × 2.5 (12.5 ms safety net)

// Rotor alignment
#define COMM_ALIGN_TIME_MS  100UL
#define COMM_ALIGN_DUTY_PCT  15

// V/f duty floor (startup torque)
#define DUTY_FLOOR_PCT       15

// Timing
#define FAULT_CHECK_INTERVAL_MS   50
#define THROTTLE_READ_INTERVAL_US 500

// Safety
#define ADC_ARM_THRESHOLD  100

// Battery (1S LiPo / LiHV)
#define BAT_THR_1S_MIN  3000
#define BAT_THR_1S_LOW  3300
#define BAT_THR_1S_MAX  4500

/*================================================================
 * State
 *===============================================================*/

static drv8311_handle_t drv8311 = NULL;

enum MotorState : uint8_t {
    STATE_IDLE = 0,
    STATE_ALIGN,
    STATE_RUN,
};
static MotorState motorState = STATE_IDLE;

// Commutation
static uint8_t  commStep       = 0;
static uint32_t lastStepUs     = 0;

// Alignment
static uint32_t alignStartMs   = 0;

// Current-assisted commutation
static uint16_t stepPeakI      = 0;   // running max LOW-phase ADC this step
static uint8_t  peakStep       = 0;   // commStep when peak was captured

// Throttle
static uint16_t throttleAdc    = 0;
static uint32_t lastThrottleUs = 0;

// Periodic tasks
static uint32_t lastFaultMs    = 0;

// Battery
enum BattState : uint8_t { BATT_OK = 0, BATT_LOW, BATT_ERR };
static BattState battState = BATT_OK;
static uint32_t  vBattMv   = 0;

// Fault latch
static bool faultLatched = false;

/*================================================================
 * LED
 *===============================================================*/

#define LED_ON()   digitalWrite(LED_PIN, LOW)   // active-low
#define LED_OFF()  digitalWrite(LED_PIN, HIGH)

/*================================================================
 * Commutation table (6-step trapezoidal, forward)
 *===============================================================*/

typedef struct {
    DRV8311_PHASE_MODE_t u, v, w;
    uint8_t  duty_reg;          // 0=A, 1=B, 2=C
} comm_step_t;

static const comm_step_t commTable[6] = {
    // U (A)           V (B)           W (C)           duty_reg
    { PHASE_CMP_PWM,   PHASE_OFF,      PHASE_SET_LOW,  0 },  // step 0: A→C
    { PHASE_OFF,       PHASE_CMP_PWM,  PHASE_SET_LOW,  1 },  // step 1: B→C
    { PHASE_SET_LOW,   PHASE_CMP_PWM,  PHASE_OFF,      1 },  // step 2: B→A
    { PHASE_SET_LOW,   PHASE_OFF,      PHASE_CMP_PWM,  2 },  // step 3: C→A
    { PHASE_OFF,       PHASE_SET_LOW,  PHASE_CMP_PWM,  2 },  // step 4: C→B
    { PHASE_CMP_PWM,   PHASE_SET_LOW,  PHASE_OFF,      0 },  // step 5: A→B
};

static const uint8_t dutyRegAddr[3] = {
    DRV8311_PWMG_A_DUTY_ADDR,
    DRV8311_PWMG_B_DUTY_ADDR,
    DRV8311_PWMG_C_DUTY_ADDR,
};

// Which CSA pin reads the LOW phase for each step
static const uint8_t lowCsaPin[6] = {
    ISEN_W_PIN, ISEN_W_PIN,        // steps 0-1: W is LOW
    ISEN_U_PIN, ISEN_U_PIN,        // steps 2-3: U is LOW
    ISEN_V_PIN, ISEN_V_PIN,        // steps 4-5: V is LOW
};

/*================================================================
 * Helpers
 *===============================================================*/

static void motorStop(void)
{
    motorState = STATE_IDLE;
    if (drv8311 != NULL)
        drv8311_phase_ctrl(drv8311, PHASE_OFF, PHASE_OFF, PHASE_OFF);
}

static void motorCommute(uint16_t duty)
{
    const comm_step_t *s = &commTable[commStep];
    drv8311_phase_ctrl(drv8311, s->u, s->v, s->w);
    drv8311_set_duty_single(drv8311, dutyRegAddr[s->duty_reg], duty);
    commStep = (commStep + 1) % 6;
}

// Throttle ADC → step period (hyperbolic: period × duty_pct ≈ 11000)
static uint32_t adcToPeriodUs(uint16_t adc)
{
    if (adc <= ADC_ARM_THRESHOLD) return 0;
    uint32_t pct = (uint32_t)map((long)adc, 0L, (long)ADC_MAX_VALUE, 1L, 100L);
    uint32_t p   = 11000UL / pct;
    if (p > COMM_STEP_SLOW_US) p = COMM_STEP_SLOW_US;
    if (p < COMM_STEP_FAST_US) p = COMM_STEP_FAST_US;
    return p;
}

// Step period → PWM duty (inverse: period × duty_pct ≈ 11000)
static uint16_t periodToDuty(uint32_t periodUs)
{
    uint32_t d = ((uint32_t)DRV8311_PWM_PERIOD * 110UL) / periodUs;
    if (d > (uint32_t)DRV8311_PWM_PERIOD)
        d = DRV8311_PWM_PERIOD;
    if (d < (uint32_t)(DRV8311_PWM_PERIOD * DUTY_FLOOR_PCT / 100))
        d = (uint32_t)(DRV8311_PWM_PERIOD * DUTY_FLOOR_PCT / 100);
    return (uint16_t)d;
}

/*================================================================
 * Battery
 *===============================================================*/

static bool lowBatteryCheck(void)
{
    const uint16_t raw = analogRead(BATT_PIN);
    vBattMv = ((uint64_t)raw * 147ULL * VOLTAGE_REFERENCE_MILLIVOLTS)
            / (47ULL * ADC_MAX_VALUE);

    battState = BATT_OK;

    if      (vBattMv <= BAT_THR_1S_MIN)  battState = BATT_ERR;
    else if (vBattMv <= BAT_THR_1S_LOW)  battState = BATT_LOW;
    else if (vBattMv <= BAT_THR_1S_MAX)  battState = BATT_OK;
    else                                  battState = BATT_ERR;

    return (battState == BATT_LOW);
}

/*================================================================
 * DRV8311 helpers
 *===============================================================*/

static void spiTransmitCb(uint8_t *send_data, uint8_t send_len,
                          uint8_t *rec_data,  uint8_t rec_len)
{
    GPIO_ResetBits(GPIOC, GPIO_Pin_3);           // CS low
    for (uint8_t i = 0; i < send_len; i++) {
        uint8_t tx = send_data[i], rx = 0;
        for (int8_t b = 7; b >= 0; b--) {
            if (tx & (1 << b))
                GPIO_SetBits(GPIOC, GPIO_Pin_6);      // MOSI
            else
                GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            GPIO_SetBits(GPIOC, GPIO_Pin_5);            // SCK ↑
            if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
                rx |= (1 << b);                         // MISO
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);          // SCK ↓
        }
        rec_data[i] = rx;
    }
    GPIO_SetBits(GPIOC, GPIO_Pin_3);             // CS high
}

static void nsleepSetCb(uint8_t level)
{
    digitalWrite(DRV8311_nSLEEP_PIN, level ? HIGH : LOW);
}

static bool drvFaultCheck(void)
{
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) != Bit_RESET)
        return false;
    drv8311_dev_sts1_t sts = drv8311_get_status(drv8311);
    (void)sts;  // nFAULT asserted = fault regardless of SPI detail
    return true;
}

/*================================================================
 * Setup
 *===============================================================*/

void setup()
{
    // -- Hold DRV asleep --
    pinMode(DRV8311_nSLEEP_PIN, OUTPUT);
    digitalWrite(DRV8311_nSLEEP_PIN, LOW);

    // -- Boot LED ON 1 s --
    pinMode(LED_PIN, OUTPUT);
    LED_ON();
    delay(1000);
    LED_OFF();
    delay(500);

    // -- GPIO --
    pinMode(ADC_PIN,              INPUT);
    pinMode(BATT_PIN,             INPUT);
    pinMode(DRV8311_nFAULT_PIN,   INPUT);
    pinMode(DRV8311_CS_PIN,       OUTPUT);
    pinMode(DRV8311_SCK_PIN,      OUTPUT);
    pinMode(DRV8311_MOSI_PIN,     OUTPUT);
    pinMode(DRV8311_MISO_PIN,     INPUT);
    pinMode(ISEN_U_PIN,           INPUT);
    pinMode(ISEN_V_PIN,           INPUT);
    pinMode(ISEN_W_PIN,           INPUT);
    pinMode(PD3, OUTPUT);  digitalWrite(PD3, LOW);  // PWM_SYNC quiet

    digitalWrite(DRV8311_CS_PIN,  HIGH);
    digitalWrite(DRV8311_SCK_PIN, LOW);
    digitalWrite(DRV8311_MOSI_PIN,LOW);
    delay(200);   // bulk cap charge

    // -- DRV8311 init --
    drv8311_cfg_t cfg = {
        .pwmcnt_mode   = UP,
        .sync_mode     = SYNC_DISABLE,
        .spi_clk       = SPI_FREQ_1M,
        .spi_sync_clks = CLOCKS_512,
        .portal        = SPI,
        .csa_gain      = CSA_GAIN,
        .pwm_period    = DRV8311_PWM_PERIOD,
        .use_csa       = 0,
        .dev_id        = 0,
        .parity_check  = 0,
        .spi_trans     = spiTransmitCb,
        .nsleep_set    = nsleepSetCb,
    };
    if (drv8311_init(&drv8311, &cfg) != 0) {
        while (1) { LED_ON(); delay(100); LED_OFF(); delay(100); }
    }

    // -- Quick hardware check --
    uint16_t r;
    r = drv8311_read_reg(drv8311, DRV8311_DEV_STS1_ADDR);
    if (r == 0xFFFF) goto fault_halt;
    r = drv8311_read_reg(drv8311, DRV8311_SYS_CTRL_ADDR);
    if (r & 0x0080)  goto fault_halt;               // REG_LOCK still set

    drv8311_out_ctrl(drv8311, 1);                    // pwm_en = 1
    r = drv8311_read_reg(drv8311, DRV8311_PWMG_CTRL_ADDR);
    if (!(r & 0x0400)) goto fault_halt;              // pwm_en didn't stick

    drv8311_phase_ctrl(drv8311, PHASE_OFF, PHASE_OFF, PHASE_OFF);
    r = drv8311_read_reg(drv8311, DRV8311_PWM_STATE_ADDR);
    if (r != 0x0000) goto fault_halt;

    goto init_ok;

fault_halt:
    while (1) { LED_ON(); delay(100); LED_OFF(); delay(100); }

init_ok:
    // -- Arm: LED ON, wait for throttle ≤ threshold --
    LED_ON();
    while (analogRead(ADC_PIN) > ADC_ARM_THRESHOLD)
        delay(10);

    // -- Ready --
    motorState     = STATE_IDLE;
    lastStepUs     = micros();
    lastFaultMs    = millis();
    lastThrottleUs = micros();
    faultLatched   = false;

    LED_OFF();
}

/*================================================================
 * Loop
 *===============================================================*/

void loop()
{
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();

    // ---- throttle read (throttled) ----
    if (nowUs - lastThrottleUs >= THROTTLE_READ_INTERVAL_US) {
        lastThrottleUs = nowUs;
        throttleAdc = analogRead(ADC_PIN);
    }
    uint32_t targetPeriod = adcToPeriodUs(throttleAdc);

    // ---- periodic fault / battery check ----
    if (nowMs - lastFaultMs >= FAULT_CHECK_INTERVAL_MS) {
        lastFaultMs = nowMs;
        lowBatteryCheck();  // updates battState, vBattMv
        if (drvFaultCheck() || battState == BATT_ERR) {
            motorStop();
            faultLatched = true;
        }
    }
    // Low battery = warning: halve throttle, don't stop
    if (battState == BATT_LOW)
        throttleAdc = throttleAdc / 2;

    // ---- fault halt ----
    if (faultLatched) {
        LED_ON();  delay(100);
        LED_OFF(); delay(100);
        return;
    }

    // ---- state machine ----
    switch (motorState) {

    case STATE_IDLE:
        if (targetPeriod == 0) break;   // throttle off — stay idle

        // Throttle on → align rotor
        motorState   = STATE_ALIGN;
        commStep     = 0;
        alignStartMs = nowMs;
        {
            const comm_step_t *s = &commTable[0];
            uint16_t ad = (uint16_t)(DRV8311_PWM_PERIOD * COMM_ALIGN_DUTY_PCT / 100);
            drv8311_phase_ctrl(drv8311, s->u, s->v, s->w);
            drv8311_set_duty_single(drv8311, dutyRegAddr[s->duty_reg], ad);
        }
        break;

    case STATE_ALIGN:
        if (targetPeriod == 0) { motorStop(); break; }  // throttle off

        if (nowMs - alignStartMs >= COMM_ALIGN_TIME_MS) {
            // Alignment done — enter current-assisted running
            motorState  = STATE_RUN;
            stepPeakI   = 0;
            peakStep    = 0xFF;          // force reset on first iter
            lastStepUs  = nowUs;

            // first commutation step
            motorCommute(periodToDuty(COMM_STEP_SLOW_US));
        }
        break;

    case STATE_RUN: {
        if (targetPeriod == 0) { motorStop(); break; }

        // Detect step change → reset peak tracker
        if (peakStep != commStep) {
            stepPeakI = 0;
            peakStep  = commStep;
        }

        // Read LOW-phase CSA
        uint16_t csa = analogRead(lowCsaPin[commStep]);
        if (csa > stepPeakI) stepPeakI = csa;

        // Should we commutate?
        bool go = false;

        // (A) Current-drop trigger
        if (stepPeakI > CURRENT_NOISE_FLOOR) {
            uint16_t thr = (uint16_t)((uint32_t)stepPeakI * CURRENT_ASSIST_PCT / 100);
            if (csa < thr) go = true;
        }

        // (B) Timeout: SLOW × 2.5 (safety net — current-drop fires earlier)
        uint32_t tout = COMM_STEP_SLOW_US * TIMEOUT_MARGIN_PCT / 100;
        if (nowUs - lastStepUs >= tout) go = true;

        if (go) {
            lastStepUs = nowUs;
            motorCommute(periodToDuty(targetPeriod));
        }
        break;
    }
    }
}
