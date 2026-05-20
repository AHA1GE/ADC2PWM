#include "CH32V003_SERVO.h"

#define PWM_PIN SERVO_PIN_PC4_TIM1_CH4

static Servo esc;

static uint16_t adcToPulseUs(uint16_t adcValue) {
	return (uint16_t)map((long)adcValue, 0L, (long)1023, 1000, 2000);
}

void setup() {
    esc.attach(PWM_PIN);
	esc.writeMicroseconds(890);
    while (analogRead(PA2) > 500 ){
        continue;
    }
}

void loop() {
    esc.writeMicroseconds(adcToPulseUs(analogRead(PA2)));
}
