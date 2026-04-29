#ifndef ServoTimers_h
#define ServoTimers_h

/*
 * Minimal CH32 timer definitions for this Servo port.
 *
 * The CH32 backend intentionally supports only two virtual pins:
 *   SERVO_PIN_CH1 -> PD4 / TIM2_CH1
 *   SERVO_PIN_CH4 -> PD7 / TIM2_CH4
 */
#define _useTimer2
typedef enum { _timer2, _Nbr_16timers } timer16_Sequence_t;

#define SERVO_PIN_CH1 0
#define SERVO_PIN_CH4 1

#endif
