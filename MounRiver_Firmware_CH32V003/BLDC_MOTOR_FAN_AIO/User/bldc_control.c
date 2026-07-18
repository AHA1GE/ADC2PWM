/*===========================================================================
 * bldc_control.c — BLDC Sensorless Control for BLDC_MOTOR_FAN_AIO
 *
 * BEMF zero-crossing detection with DRV8311 tSPI duty control.
 * 6-step trapezoidal commutation, open-loop startup → closed-loop BEMF.
 *===========================================================================*/

#include "bldc_control.h"
#include "bldc_hal.h"
#include "drv8311.h"
#include <stddef.h>   // NULL

/*===========================================================================
 * Commutation Table
 *
 * Uses PHASE_HIGH_PWM (not CMP_PWM) so the floating phase can be measured
 * during PWM OFF time via the low-side body diode.
 *===========================================================================*/

const CommEntry_t comm_table[6] = {
    // Step 0: U=HIGH_PWM, V=OFF,      W=SET_LOW   → sense W (CH7/A7/PD4), BEMF RISING
    { PHASE_HIGH_PWM, PHASE_OFF,      PHASE_SET_LOW, ADC_CH_BEMF_W, 1 },
    // Step 1: U=OFF,      V=HIGH_PWM, W=SET_LOW   → sense U (CH5/A5/PD5), BEMF FALLING
    { PHASE_OFF,      PHASE_HIGH_PWM, PHASE_SET_LOW, ADC_CH_BEMF_U, 0 },
    // Step 2: U=SET_LOW,  V=HIGH_PWM, W=OFF       → sense U (CH5/A5/PD5), BEMF RISING
    { PHASE_SET_LOW,  PHASE_HIGH_PWM, PHASE_OFF,      ADC_CH_BEMF_U, 1 },
    // Step 3: U=SET_LOW,  V=OFF,      W=HIGH_PWM  → sense V (CH6/A6/PD6), BEMF FALLING
    { PHASE_SET_LOW,  PHASE_OFF,      PHASE_HIGH_PWM, ADC_CH_BEMF_V, 0 },
    // Step 4: U=OFF,      V=SET_LOW,  W=HIGH_PWM  → sense V (CH6/A6/PD6), BEMF RISING
    { PHASE_OFF,      PHASE_SET_LOW,  PHASE_HIGH_PWM, ADC_CH_BEMF_V, 1 },
    // Step 5: U=HIGH_PWM, V=SET_LOW,  W=OFF       → sense W (CH7/A7/PD4), BEMF FALLING
    { PHASE_HIGH_PWM, PHASE_SET_LOW,  PHASE_OFF,      ADC_CH_BEMF_W, 0 },
};

/*===========================================================================
 * Global Motor Control Instance
 *===========================================================================*/

MotorCtrl_t motor;

// Last TIM1 tick snapshot for BEMF sampling rate-limiting
static uint32_t last_bemf_tick;

/*===========================================================================
 * Initialization
 *===========================================================================*/

void BLDC_Init(void)
{
    motor.state = STATE_IDLE;
    motor.step = STEP_0;
    motor.throttle_pct = 0;
    motor.duty_raw = 0;
    motor.fault_latched = 0;
    motor.openloop_count = 0;
    motor.bemf_ready = 0;
    motor.zc_detected = 0;
    motor.bemf_ema = 0;
    motor.neutral_ema = 0;
    motor.ema_samples = 0;
}

/*===========================================================================
 * State Transitions
 *===========================================================================*/

void BLDC_EnterIdle(void)
{
    DRV8311_Disable();
    HAL_TIM1_Stop();
    // Set all phases off for safety
    if (drv) {
        drv8311_phase_ctrl(drv, PHASE_OFF, PHASE_OFF, PHASE_OFF);
        drv8311_set_duty_raw(drv, 0, 0, 0);
    }
    motor.state = STATE_IDLE;
    motor.throttle_pct = 0;
    motor.duty_raw = 0;
    motor.zc_detected = 0;
    motor.bemf_ready = 0;
    motor.ema_samples = 0;
}

void BLDC_EnterAlign(void)
{
    if (!drv) return;

    DRV8311_Enable();
    HAL_TIM1_Start();

    motor.state = STATE_ALIGN;
    motor.step = STEP_0;
    motor.duty_raw = DRV_PWM_ALIGN_DUTY;
    motor.align_start_millis = sys_tick_millis;
    motor.openloop_count = OPENLOOP_FORCED_STEPS;
    motor.bemf_ready = 0;
    motor.zc_detected = 0;
    motor.bemf_ema = 0;
    motor.neutral_ema = 0;
    motor.ema_samples = 0;

    // Set step 0 with alignment duty
    const CommEntry_t *e = &comm_table[STEP_0];
    drv8311_phase_ctrl(drv, e->phase_a, e->phase_b, e->phase_c);
    drv8311_set_duty_raw(drv,
        (e->phase_a == PHASE_HIGH_PWM) ? motor.duty_raw : 0,
        (e->phase_b == PHASE_HIGH_PWM) ? motor.duty_raw : 0,
        (e->phase_c == PHASE_HIGH_PWM) ? motor.duty_raw : 0);

    // Initialize BEMF timing
    motor.last_step_ticks = COMM_STEP_SLOW_MICROS / 63;  // Convert µs to TIM1 ticks
    motor.step_start_ticks = tim1_tick_count;
    last_bemf_tick = tim1_tick_count;
}

void BLDC_EnterRun(void)
{
    motor.state = STATE_RUN;
    motor.bemf_ready = 0;
    motor.ema_samples = 0;
    motor.zc_detected = 0;

    // First real commutation from step 0 to step 1 at slow speed
    motor.step = STEP_1;
    BLDC_Commutate();
    motor.step_start_ticks = tim1_tick_count;
    motor.last_step_ticks = COMM_STEP_SLOW_MICROS / 63;
    last_bemf_tick = tim1_tick_count;
}

void BLDC_EnterFault(void)
{
    DRV8311_Disable();
    HAL_TIM1_Stop();
    if (drv) {
        drv8311_phase_ctrl(drv, PHASE_OFF, PHASE_OFF, PHASE_OFF);
        drv8311_set_duty_raw(drv, 0, 0, 0);
    }
    motor.state = STATE_FAULT;
    motor.fault_latched = 1;
    motor.throttle_pct = 0;
    motor.duty_raw = 0;
}

/*===========================================================================
 * Commutation Execution
 *
 * Writes phase states and duty cycles to DRV8311 via tSPI.
 * Only the HIGH_PWM phase gets duty; the SET_LOW phase gets 0;
 * the OFF phase gets 0.
 *===========================================================================*/

void BLDC_Commutate(void)
{
    if (!drv) return;

    const CommEntry_t *e = &comm_table[motor.step];

    drv8311_phase_ctrl(drv, e->phase_a, e->phase_b, e->phase_c);

    uint16_t da = (e->phase_a == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
    uint16_t db = (e->phase_b == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
    uint16_t dc = (e->phase_c == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
    drv8311_set_duty_raw(drv, da, db, dc);

    // Reset BEMF detection for the new step
    motor.zc_detected = 0;
    motor.bemf_ema = 0;
    motor.neutral_ema = 0;
    motor.ema_samples = 0;

    // Update duty from throttle (in case it changed during this step)
    motor.duty_raw = (uint16_t)((uint32_t)motor.throttle_pct * DRV_PWM_MAX_DUTY / THROTTLE_MAX_PCT);
    if (motor.duty_raw < PWM_MIN_DUTY && motor.duty_raw > 0)
        motor.duty_raw = PWM_MIN_DUTY;
}

/*===========================================================================
 * Throttle from Buttons
 *
 * Btn_Up:   single click +10%, long press +1% every 100ms
 * Btn_Down: single click -10%, long press -1% every 100ms
 * Always starts at 0%.
 *===========================================================================*/

#include "bldc_hal.h"  // BTN_Up_Pressed, BTN_Down_Pressed, BTN_*_IsLongPress

void BLDC_ProcessButtons(void)
{
    uint8_t changed = 0;

    if (BTN_Up_Pressed()) {
        if (BTN_Up_IsLongPress()) {
            // Fine step: +1%
            if (motor.throttle_pct < THROTTLE_MAX_PCT) {
                motor.throttle_pct++;
                changed = 1;
            }
        } else {
            // Coarse step: +10%
            if (motor.throttle_pct + 10 <= THROTTLE_MAX_PCT)
                motor.throttle_pct += 10;
            else
                motor.throttle_pct = THROTTLE_MAX_PCT;
            changed = 1;
        }
    }

    if (BTN_Down_Pressed()) {
        if (BTN_Down_IsLongPress()) {
            // Fine step: -1%
            if (motor.throttle_pct > 0) {
                motor.throttle_pct--;
                changed = 1;
            }
        } else {
            // Coarse step: -10%
            if (motor.throttle_pct >= 10)
                motor.throttle_pct -= 10;
            else
                motor.throttle_pct = 0;
            changed = 1;
        }
    }

    if (changed && motor.state == STATE_RUN) {
        // Immediately update duty
        motor.duty_raw = (uint16_t)((uint32_t)motor.throttle_pct * DRV_PWM_MAX_DUTY / THROTTLE_MAX_PCT);
        if (motor.duty_raw < PWM_MIN_DUTY && motor.duty_raw > 0)
            motor.duty_raw = PWM_MIN_DUTY;
        // Apply to current step (update PWM phase duty)
        const CommEntry_t *e = &comm_table[motor.step];
        uint16_t da = (e->phase_a == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
        uint16_t db = (e->phase_b == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
        uint16_t dc = (e->phase_c == PHASE_HIGH_PWM) ? motor.duty_raw : 0;
        drv8311_set_duty_raw(drv, da, db, dc);
    }
}

/*===========================================================================
 * BEMF Sampling + Zero-Crossing Detection
 *
 * Called from main loop at ~250µs intervals (every 4 TIM1 ticks).
 * Samples the floating phase BEMF and neutral, runs EMA filter,
 * detects ZC, and triggers commutation after 30° delay.
 *===========================================================================*/

void BLDC_SampleBEMF(void)
{
    if (motor.state != STATE_RUN) return;

    const CommEntry_t *e = &comm_table[motor.step];
    uint16_t bemf_raw, neutral_raw;

    // Sample floating phase BEMF
    ADC_RegularChannelConfig(ADC1, e->bemf_adc_ch, 1, ADC_SampleTime_73Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    bemf_raw = ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    // Sample neutral
    ADC_RegularChannelConfig(ADC1, ADC_CH_BEMF_NEUTRAL, 1, ADC_SampleTime_73Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    neutral_raw = ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    // EMA filter: α = 1/8 → ema = (7 * ema + sample) / 8
    // Fixed-point: ×256
    if (motor.ema_samples == 0) {
        motor.bemf_ema = bemf_raw * 256;
        motor.neutral_ema = neutral_raw * 256;
        motor.ema_samples = 1;
    } else {
        motor.bemf_ema = ((motor.bemf_ema * 7) >> 3) + (bemf_raw * 32);
        motor.neutral_ema = ((motor.neutral_ema * 7) >> 3) + (neutral_raw * 32);
        if (motor.ema_samples < 255) motor.ema_samples++;
    }

    // Check if BEMF signal is strong enough for closed-loop
    if (motor.ema_samples >= 4 && !motor.bemf_ready) {
        uint16_t bemf = motor.bemf_ema >> 8;
        uint16_t neutral = motor.neutral_ema >> 8;
        int16_t diff = (int16_t)bemf - (int16_t)neutral;
        if (diff < 0) diff = -diff;
        if (diff >= BEMF_SWING_THRESHOLD)
            motor.bemf_ready = 1;
    }

    // Zero-crossing detection (only in closed-loop, or late open-loop)
    if (!motor.zc_detected && motor.ema_samples >= 4) {
        uint16_t bemf = motor.bemf_ema >> 8;
        uint16_t neutral = motor.neutral_ema >> 8;
        uint8_t crossing = 0;

        if (e->expect_rising) {
            // Rising BEMF: ZC when bemf crosses above neutral
            if (bemf >= neutral)
                crossing = 1;
        } else {
            // Falling BEMF: ZC when bemf crosses below neutral
            if (bemf <= neutral)
                crossing = 1;
        }

        if (crossing) {
            motor.zc_detected = 1;
            motor.zc_ticks = tim1_tick_count;
        }
    }
}

/*===========================================================================
 * Commutation Timing Check
 *
 * Called from main loop (inside BLDC_StateMachine or separately).
 * Checks if it's time to commutate based on ZC + 30° delay, or timeout.
 *===========================================================================*/

static void BLDC_CheckCommutation(void)
{
    if (motor.state != STATE_RUN) return;

    uint32_t now = tim1_tick_count;
    uint8_t should_commutate = 0;

    if (motor.zc_detected && motor.bemf_ready) {
        // Closed-loop: wait 30 electrical degrees after ZC
        // delay_30_ticks = last_step_ticks / 2 (30° = 60° / 2)
        uint32_t delay_30_ticks = motor.last_step_ticks / ZC_DELAY_DIVISOR;
        if (delay_30_ticks < 2) delay_30_ticks = 2;  // Min ~126µs

        uint32_t elapsed = now - motor.zc_ticks;
        if (elapsed >= delay_30_ticks)
            should_commutate = 1;
    }
    else {
        // Open-loop or timeout: use fixed timing
        uint32_t timeout_ticks = motor.last_step_ticks * COMM_TIMEOUT_MARGIN;
        uint32_t elapsed = now - motor.step_start_ticks;

        if (motor.openloop_count > 0) {
            // Open-loop: use fixed slow timing
            uint32_t openloop_ticks = COMM_STEP_SLOW_MICROS / 63;
            if (elapsed >= openloop_ticks) {
                should_commutate = 1;
            }
        } else if (motor.bemf_ready && elapsed >= timeout_ticks) {
            // Timeout safety net: force commutate
            should_commutate = 1;
        }
    }

    if (should_commutate) {
        // Update step timing
        motor.last_step_ticks = now - motor.step_start_ticks;
        motor.step_start_ticks = now;

        // Advance step
        motor.step = (motor.step + 1) % 6;

        if (motor.openloop_count > 0)
            motor.openloop_count--;

        BLDC_Commutate();
    }
}

/*===========================================================================
 * State Machine — called from main loop
 *===========================================================================*/

void BLDC_StateMachine(void)
{
    switch (motor.state) {

    case STATE_IDLE:
        if (motor.throttle_pct > 0) {
            BLDC_EnterAlign();
        }
        break;

    case STATE_ALIGN: {
        uint32_t elapsed = sys_tick_millis - motor.align_start_millis;

        if (motor.throttle_pct == 0) {
            BLDC_EnterIdle();
        } else if (elapsed >= COMM_ALIGN_TIME_MILLIS) {
            // Set target duty before entering RUN
            motor.duty_raw = (uint16_t)((uint32_t)motor.throttle_pct * DRV_PWM_MAX_DUTY / THROTTLE_MAX_PCT);
            if (motor.duty_raw < PWM_MIN_DUTY) motor.duty_raw = PWM_MIN_DUTY;
            BLDC_EnterRun();
        }
        break;
    }

    case STATE_RUN:
        if (motor.throttle_pct == 0) {
            BLDC_EnterIdle();
        } else {
            // BEMF sampling at ~250µs (every 4 TIM1 ticks)
            if (tim1_tick_count - last_bemf_tick >= 4) {
                last_bemf_tick = tim1_tick_count;
                BLDC_SampleBEMF();
            }
            // Check if it's time to commutate
            BLDC_CheckCommutation();
        }
        break;

    case STATE_FAULT:
        // Latched — no automatic recovery
        // LED blinking handled in main.c
        break;
    }
}

/*===========================================================================
 * Bus Current Estimation (from CSA ADC readings)
 *
 * Total bus current ≈ average of all 3 CSA outputs (when each is active).
 * DRV8311 CSA gain = 0.5V/A. V_ADC = ADC * 3.3 / 1024.
 * I = V_ADC / 0.5 = ADC * 3.3 / 1024 / 0.5 = ADC * 0.006445 A
 *   = ADC * 6.445 mA
 * We average all 3 channels and divide by 3.
 *===========================================================================*/

uint16_t BLDC_GetBusCurrent_mA(void)
{
    uint32_t sum = 0;

    // Sample all 3 CSA channels
    ADC_RegularChannelConfig(ADC1, ADC_CH_CURR_U, 1, ADC_SampleTime_73Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    sum += ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    ADC_RegularChannelConfig(ADC1, ADC_CH_CURR_V, 1, ADC_SampleTime_73Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    sum += ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    ADC_RegularChannelConfig(ADC1, ADC_CH_CURR_W, 1, ADC_SampleTime_73Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    sum += ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    // Average, then scale: mA = avg * 3300 / 1024 / 0.5 = avg * 6.445
    uint16_t avg = (uint16_t)(sum / 3);
    return (uint16_t)((uint32_t)avg * 6445 / 1000);
}
