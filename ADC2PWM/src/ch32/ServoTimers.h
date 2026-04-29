#ifndef ServoTimers_h
#define ServoTimers_h

/*
 * Minimal CH32 timer definitions for this Servo port.
 *
 * The CH32 backend intentionally supports only below pins:
 *   PD4_TIM2CH1 -> PD4 / TIM2_CH1
 *   PD7_TIM2CH4 -> PD7 / TIM2_CH4
 *   PC4_TIM1CH4 -> PC4 / TIM1_CH4
 */
#define _useTimer2
typedef enum { _timer2, _Nbr_16timers } timer16_Sequence_t;

#define PD4_TIM2CH1 0 // PD4 / TIM2_CH1
#define PD7_TIM2CH4 1 // PD7 / TIM2_CH4
#define PC4_TIM1CH4 2 // PC4 / TIM1_CH4

#endif
