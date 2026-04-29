#if defined(ARDUINO_ARCH_CH32) || defined(ARDUINO_ARCH_CH32V) || defined(CH32V003) || defined(CH32V003xx) || defined(CH32V003F4) || defined(CH32V00x) || defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x) || defined(CH32X035)

#include <Arduino.h>

#include "../Servo.h"

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)

#define SERVO_TIM2_PSC        47u
#define SERVO_TIM2_ARR        19999u
#define SERVO_HARD_MIN_US     500u
#define SERVO_HARD_MAX_US     2500u
#define SERVO_DEFAULT_US      ((uint16_t)DEFAULT_PULSE_WIDTH)

static servo_t servos[MAX_SERVOS];
uint8_t ServoCount = 0;

static bool s_timerInitialized = false;
static uint8_t s_channelOwner[2] = { INVALID_SERVO, INVALID_SERVO };
static uint16_t s_channelPulseUs[2] = { SERVO_DEFAULT_US, SERVO_DEFAULT_US };

static bool isValidVirtualPin(int pin)
{
	return (pin == PD4_TIM2CH1) || (pin == PD7_TIM2CH4);
}

static uint8_t pinToSlot(int pin)
{
	return (pin == PD4_TIM2CH1) ? 0 : 1;
}

static uint16_t clampPulseUs(uint16_t us)
{
	if (us < SERVO_HARD_MIN_US) {
		return SERVO_HARD_MIN_US;
	}
	if (us > SERVO_HARD_MAX_US) {
		return SERVO_HARD_MAX_US;
	}
	return us;
}

static void initTimer2IfNeeded(void)
{
	if (s_timerInitialized) {
		return;
	}
	s_timerInitialized = true;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef gpio = {0};
	gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &gpio);

	TIM_TimeBaseInitTypeDef tb = {0};
	tb.TIM_Period = SERVO_TIM2_ARR;
	tb.TIM_Prescaler = SERVO_TIM2_PSC;
	tb.TIM_ClockDivision = TIM_CKD_DIV1;
	tb.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &tb);

	TIM_OCInitTypeDef oc = {0};
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = SERVO_DEFAULT_US;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &oc);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM2, &oc);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
	TIM_Cmd(TIM2, ENABLE);
}

static void writePulseToTimer(int pin, uint16_t pulseUs)
{
	const uint16_t clamped = clampPulseUs(pulseUs);
	if (pin == PD4_TIM2CH1) {
		TIM_SetCompare1(TIM2, clamped);
		s_channelPulseUs[0] = clamped;
	} else {
		TIM_SetCompare4(TIM2, clamped);
		s_channelPulseUs[1] = clamped;
	}
}

static uint16_t readPulseFromCache(int pin)
{
	return s_channelPulseUs[pinToSlot(pin)];
}

Servo::Servo()
{
	if (ServoCount < MAX_SERVOS) {
		this->servoIndex = ServoCount++;
		servos[this->servoIndex].ticks = SERVO_DEFAULT_US;
		servos[this->servoIndex].Pin.isActive = false;
		servos[this->servoIndex].Pin.nbr = 0;
	} else {
		this->servoIndex = INVALID_SERVO;
	}
}

uint8_t Servo::attach(int pin)
{
	return this->attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t Servo::attach(int pin, int min, int max)
{
	if (this->servoIndex == INVALID_SERVO) {
		return INVALID_SERVO;
	}

	if (!isValidVirtualPin(pin)) {
		return INVALID_SERVO;
	}

	const uint8_t newSlot = pinToSlot(pin);
	if ((s_channelOwner[newSlot] != INVALID_SERVO) && (s_channelOwner[newSlot] != this->servoIndex)) {
		return INVALID_SERVO;
	}

	if (servos[this->servoIndex].Pin.isActive) {
		const int oldPin = servos[this->servoIndex].Pin.nbr;
		if (isValidVirtualPin(oldPin)) {
			const uint8_t oldSlot = pinToSlot(oldPin);
			if (s_channelOwner[oldSlot] == this->servoIndex) {
				s_channelOwner[oldSlot] = INVALID_SERVO;
			}
		}
	}

	initTimer2IfNeeded();

	this->min = (MIN_PULSE_WIDTH - min) / 4;
	this->max = (MAX_PULSE_WIDTH - max) / 4;
	servos[this->servoIndex].Pin.nbr = (uint8_t)pin;
	servos[this->servoIndex].Pin.isActive = true;
	s_channelOwner[newSlot] = this->servoIndex;

	this->writeMicroseconds(SERVO_DEFAULT_US);
	return this->servoIndex;
}

void Servo::detach()
{
	if ((this->servoIndex == INVALID_SERVO) || !servos[this->servoIndex].Pin.isActive) {
		return;
	}

	const int pin = servos[this->servoIndex].Pin.nbr;
	if (isValidVirtualPin(pin)) {
		const uint8_t slot = pinToSlot(pin);
		if (s_channelOwner[slot] == this->servoIndex) {
			s_channelOwner[slot] = INVALID_SERVO;
		}
	}

	servos[this->servoIndex].Pin.isActive = false;
}

void Servo::write(int value)
{
	if (value < MIN_PULSE_WIDTH) {
		if (value < 0) {
			value = 0;
		} else if (value > 180) {
			value = 180;
		}
		value = map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
	}

	this->writeMicroseconds(value);
}

void Servo::writeMicroseconds(int value)
{
	if ((this->servoIndex == INVALID_SERVO) || !servos[this->servoIndex].Pin.isActive) {
		return;
	}

	if (value < SERVO_MIN()) {
		value = SERVO_MIN();
	} else if (value > SERVO_MAX()) {
		value = SERVO_MAX();
	}

	const uint16_t pulseUs = (uint16_t)value;
	servos[this->servoIndex].ticks = pulseUs;
	writePulseToTimer(servos[this->servoIndex].Pin.nbr, pulseUs);
}

int Servo::read()
{
	return map(this->readMicroseconds(), SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int Servo::readMicroseconds()
{
	if ((this->servoIndex == INVALID_SERVO) || !servos[this->servoIndex].Pin.isActive) {
		return 0;
	}

	return (int)readPulseFromCache(servos[this->servoIndex].Pin.nbr);
}

bool Servo::attached()
{
	return (this->servoIndex != INVALID_SERVO) && servos[this->servoIndex].Pin.isActive;
}

#endif
