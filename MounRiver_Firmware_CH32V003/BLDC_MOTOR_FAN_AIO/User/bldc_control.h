#ifndef __BLDC_CONTROL_H
#define __BLDC_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "drv8311_reg.h"   // DRV8311_PHASE_MODE_t

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * Timing Constants
 *===========================================================================*/

#define COMM_ALIGN_TIME_MILLIS   100    // Rotor alignment duration
#define COMM_STEP_SLOW_MICROS    8000   // Slowest open-loop step (~125Hz electrical)
#define COMM_STEP_FAST_MICROS    200    // Fastest step (~5000Hz electrical)
#define COMM_TIMEOUT_MARGIN      3      // Timeout = last_step * 3 (safety)
#define ZC_DELAY_DIVISOR         2      // 30 electrical degrees = last_step / 2
#define ZC_DELAY_MIN_MICROS      50     // Minimum 30-degree delay

// BEMF EMA filter: α = 1/8 → ema = (7*ema + sample) / 8
#define BEMF_EMA_SHIFT           3

// Duty cycle mapping
#define THROTTLE_MAX_PCT         100
#define PWM_MIN_DUTY             144    // ~15% of 960
#define PWM_MAX_DUTY             960    // 100% of period

// Open-loop startup
#define OPENLOOP_FORCED_STEPS    12     // Number of forced commutations before BEMF
#define BEMF_SWING_THRESHOLD     20     // Min ADC counts swing to trust BEMF

/*===========================================================================
 * State Machine
 *===========================================================================*/

typedef enum {
    STATE_IDLE = 0,
    STATE_ALIGN,
    STATE_RUN,
    STATE_FAULT
} MotorState_t;

typedef enum {
    STEP_0 = 0,  // U=HIGH_PWM, V=OFF,      W=SET_LOW   → sense W
    STEP_1,      // U=OFF,      V=HIGH_PWM, W=SET_LOW   → sense U
    STEP_2,      // U=SET_LOW,  V=HIGH_PWM, W=OFF       → sense U
    STEP_3,      // U=SET_LOW,  V=OFF,      W=HIGH_PWM  → sense V
    STEP_4,      // U=OFF,      V=SET_LOW,  W=HIGH_PWM  → sense V
    STEP_5       // U=HIGH_PWM, V=SET_LOW,  W=OFF       → sense W
} CommStep_t;

/*===========================================================================
 * Commutation Table Entry
 *===========================================================================*/

typedef struct {
    DRV8311_PHASE_MODE_t phase_a;
    DRV8311_PHASE_MODE_t phase_b;
    DRV8311_PHASE_MODE_t phase_c;
    uint8_t bemf_adc_ch;        // ADC channel for floating phase BEMF
    uint8_t expect_rising;      // 1 = rising crossing, 0 = falling crossing
} CommEntry_t;

extern const CommEntry_t comm_table[6];

/*===========================================================================
 * Motor Control Structure
 *===========================================================================*/

typedef struct {
    MotorState_t state;
    CommStep_t   step;
    uint8_t      throttle_pct;      // 0-100
    uint16_t     duty_raw;          // Current duty in PWM counts

    // Timing (in TIM1 ticks; 1 tick ≈ 63µs at 15.87kHz)
    uint32_t     step_start_ticks;  // TIM1 count at last commutation
    uint32_t     last_step_ticks;   // Duration of previous step (in TIM1 ticks)
    uint32_t     zc_ticks;          // TIM1 count at ZC detection
    uint8_t      zc_detected;       // ZC found this step?

    // Alignment
    uint32_t     align_start_millis;

    // Open-loop startup
    uint8_t      openloop_count;    // Forced commutations remaining
    uint8_t      bemf_ready;        // BEMF signal is strong enough

    // BEMF EMA filters (×256 fixed-point)
    uint16_t     bemf_ema;
    uint16_t     neutral_ema;
    uint8_t      ema_samples;       // Number of samples in EMA

    // Fault
    uint8_t      fault_latched;
} MotorCtrl_t;

extern MotorCtrl_t motor;

/*===========================================================================
 * Public API
 *===========================================================================*/

void BLDC_Init(void);
void BLDC_StateMachine(void);

// Called from main loop when running
void BLDC_SampleBEMF(void);       // Sample BEMF + neutral, run ZC detection
void BLDC_ProcessButtons(void);    // Apply throttle changes from buttons

// State transitions
void BLDC_EnterIdle(void);
void BLDC_EnterAlign(void);
void BLDC_EnterRun(void);
void BLDC_EnterFault(void);

// Commutation
void BLDC_Commutate(void);

// Display helpers
uint16_t BLDC_GetBusCurrent_mA(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_CONTROL_H */
