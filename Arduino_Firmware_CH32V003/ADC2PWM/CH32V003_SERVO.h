#ifndef CH32V003_SERVO_H
#define CH32V003_SERVO_H

#include <stdint.h>


#define SERVO_PIN_PC4_TIM1_CH4 0  /* TIM1_CH4 on PC4, default pin for ESC output */

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * Initialize TIM1 for servo PWM on PC4 (CH4).
     *   - 50 Hz period (20 ms), 1 us resolution (PSC=47, ARR=19999 @ 48 MHz)
     *   - Channel starts at 1500 us neutral
     * Idempotent – safe to call from multiple Servo::attach() calls.
     */
    void CH32V003_SERVO_Init(void);

    /** Set pulse width for TIM1_CH4 / PC4.  Clamped to [500, 2500] us. */
    void CH32V003_SERVO_WriteCH4(uint16_t us);

#ifdef __cplusplus
} /* extern "C" */

/**
 * Drop-in replacement for the Arduino Servo class.
 *
 * Usage:
 *   Servo s;
 *   s.attach(SERVO_PIN_PC4_TIM1_CH4);          // bind to PC4 / TIM1_CH4
 *   s.writeMicroseconds(1000);        // 1000 us pulse
 */
class Servo
{
public:
    Servo() : _ch(0xFFu) {}

    /**
     * @param pin  SERVO_PIN_PC4_TIM1_CH4 (0) → PC4 / TIM1_CH4.
     *             Any other value silently maps to CH4.
     */
    void attach(uint8_t pin)
    {
        CH32V003_SERVO_Init();
        (void)pin;
        _ch = (uint8_t)SERVO_PIN_PC4_TIM1_CH4;
    }

    /** Set pulse width in microseconds.  Clamped to [500, 2500]. */
    void writeMicroseconds(uint16_t us)
    {
        if (_ch != SERVO_PIN_PC4_TIM1_CH4)
            return;

        CH32V003_SERVO_WriteCH4(us);
    }

private:
    uint8_t _ch;
};

#endif /* __cplusplus */

#endif /* CH32V003_SERVO_H */
