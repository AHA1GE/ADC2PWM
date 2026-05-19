#include "CH32V003_SERVO.h"


#define PWM_PIN SERVO_PIN_PC4_TIM1_CH4

#define PWM_BOOT_US 890

static Servo esc;


void setup() {
    esc.attach(PWM_PIN);
	esc.writeMicroseconds(PWM_BOOT_US);
}

void loop() {
}
