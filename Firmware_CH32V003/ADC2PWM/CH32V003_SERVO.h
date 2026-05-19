#ifndef CH32V003_SERVO_H
#define CH32V003_SERVO_H

#include <stdint.h>

#define SERVO_PIN_PD4_TIM2_CH1 0
#define SERVO_PIN_PD7_TIM2_CH4 1
#define SERVO_PIN_PC4_TIM1_CH4 2

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Initialize TIM2 for servo PWM on PD4 (CH1) and PD7 (CH4).
     *   - AFIO full remap: TIM2_RM = 11b
     *   - 50 Hz period (20 ms), 1 us resolution (PSC=47, ARR=19999 @ 48 MHz)
     *   - Both channels start at 1500 us neutral
     * Idempotent – safe to call from multiple Servo::attach() calls.
     */
    void CH32V003_SERVO_Init(void);

    /** Set pulse width for TIM2_CH1 / PD4.  Clamped to [500, 2500] us. */
    void CH32V003_SERVO_WriteCH1(uint16_t us);

    /** Set pulse width for TIM2_CH4 / PD7.  Clamped to [500, 2500] us. */
    void CH32V003_SERVO_WriteCH4(uint16_t us);

    /**
     * Initialize TIM1 for servo PWM on PC4 (CH4).
     *   - 50 Hz period (20 ms), 1 us resolution (PSC=47, ARR=19999 @ 48 MHz)
     *   - Channel starts at 1500 us neutral
     * Idempotent – safe to call from multiple Servo::attach() calls.
     */
    void CH32V003_SERVO_InitTIM1CH4(void);

    /** Set pulse width for TIM1_CH4 / PC4.  Clamped to [500, 2500] us. */
    void CH32V003_SERVO_WriteTIM1CH4(uint16_t us);

#ifdef __cplusplus
} /* extern "C" */

/**
 * Drop-in replacement for the Arduino Servo class.
 *
 * Usage:
 *   Servo s;
 *   s.attach(SERVO_PIN_PD4_TIM2_CH1);          // bind to PD4 / TIM2_CH1
 *   s.writeMicroseconds(1000);        // 1000 us pulse
 */
class Servo
{
public:
    Servo() : _ch(0xFFu) {}

    /**
     * @param pin  SERVO_PIN_PD4_TIM2_CH1 (0) → PD4,  SERVO_PIN_PD7_TIM2_CH4 (1) → PD7.
     *             Any other value silently maps to CH1.
     */
    void attach(uint8_t pin)
    {
        _ch = pin;
        if (pin == SERVO_PIN_PC4_TIM1_CH4)
            CH32V003_SERVO_InitTIM1CH4();
        else
            CH32V003_SERVO_Init();
    }

    /** Set pulse width in microseconds.  Clamped to [500, 2500]. */
    void writeMicroseconds(uint16_t us)
    {
        if (_ch == SERVO_PIN_PC4_TIM1_CH4)
            CH32V003_SERVO_WriteTIM1CH4(us);
        else if (_ch == SERVO_PIN_PD7_TIM2_CH4)
            CH32V003_SERVO_WriteCH4(us);
        else
            CH32V003_SERVO_WriteCH1(us);
    }

private:
    uint8_t _ch;
};

#endif /* __cplusplus */

#endif /* CH32V003_SERVO_H */
