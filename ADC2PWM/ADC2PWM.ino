/*
 * Unified ADC2PWM firmware for AVR, CH32 and ESP32.
 *
 * Behavior:
 * 1) Output PWM_BOOT_US during boot window with slow LED blink.
 * 2) Require ADC minimum once to unlock normal control.
 * 3) After unlock, map ADC to PWM range with asymmetric ramp limiting.
 */

#include "Servo.h"

#if defined(ARDUINO_ARCH_AVR)
#define ADC2PWM_PLATFORM_AVR 1
#elif defined(ARDUINO_ARCH_CH32) || defined(ARDUINO_ARCH_CH32V) || defined(CH32V003) || defined(CH32V003xx) || defined(CH32V003F4) || defined(CH32V00x) || defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x) || defined(CH32X035)
#define ADC2PWM_PLATFORM_CH32 1
#elif defined(ARDUINO_ARCH_ESP32)
#define ADC2PWM_PLATFORM_ESP32 1
#else
#error "ADC2PWM supports AVR, CH32 and ESP32 only."
#endif

#if defined(ADC2PWM_PLATFORM_AVR)
#define PWM_PIN 2
#define ADC_PIN A0
#define BATT_PIN A1
#define LED_PIN 3
#define LED_ACTIVE_HIGH 1 // arduino can push
#define ADC_MAX_VALUE 1023
#define SDA_PIN NULL
#define SCL_PIN NULL
#elif defined(ADC2PWM_PLATFORM_CH32)
#define PWM_PIN PD4_TIM2CH1
#define ADC_PIN PA2 // ADC0, potentiometer
#define BATT_PIN PD6 //ADC6, 4.7k/10k divider, 5v vRef -> `vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE`
#define LED_PIN PD4 // same physical pin as SWD on CH32V003J4M6
#define LED_ACTIVE_HIGH 0  // ch32 use open/drain to sink, active low
#define SDA_PIN PC1
#define SCL_PIN PC2
#define ADC_MAX_VALUE 1023
#elif defined(ADC2PWM_PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define PWM_PIN NULL
#define ADC_PIN NULL
#define BATT_PIN NULL
#define LED_PIN NULL
#define LED_ACTIVE_HIGH NULL
#define ADC_MAX_VALUE 4095
#define SDA_PIN NULL
#define SCL_PIN NULL
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define PWM_PIN NULL
#define ADC_PIN NULL
#define BATT_PIN NULL
#define LED_PIN NULL
#define LED_ACTIVE_HIGH NULL
#define ADC_MAX_VALUE 4095
#define SDA_PIN NULL
#define SCL_PIN NULL
#elif !defined(PWM_PIN) || !defined(ADC_PIN) || !defined(LED_PIN) || !defined(LED_ACTIVE_HIGH)
#error "Generic ESP32 requires predefined ADC_PIN, PWM_PIN, LED_PIN and LED_ACTIVE_HIGH before compiling."
#endif

#ifndef ADC_MAX_VALUE
#define ADC_MAX_VALUE 4095
#endif
#endif

// ---------------- PWM Range ----------------
#define PWM_BOOT_US 890
#define PWM_MIN_US 1000
#define PWM_MAX_US 2000

// ---------------- Timing ----------------
#define BOOT_TIME_MS 2100
#define LOOP_DELAY_MS 10

// ---------------- Soft Start ----------------
#define RAMP_STEP_US_UP 2
#define RAMP_STEP_US_DOWN 4

// ---------------- Safety Latch ----------------
#define ADC_ARM_THRESHOLD 10

static uint16_t pwmOutput = PWM_BOOT_US;
static uint16_t pwmTarget = PWM_BOOT_US;
static unsigned long bootTimeMs = 0;
static bool safetyArmed = false;

static Servo esc;

static void ledSet(bool on)
{
	const uint8_t level = LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
	digitalWrite(LED_PIN, level);
}

static void ledBlink(unsigned long intervalMs, unsigned long nowMs)
{
	ledSet(((nowMs / intervalMs) % 2u) == 0u);
}

static uint16_t readAdcClamped()
{
	int raw = analogRead(ADC_PIN);
	if (raw < 0) {
		raw = 0;
	}
	if (raw > ADC_MAX_VALUE) {
		raw = ADC_MAX_VALUE;
	}
	return (uint16_t)raw;
}

static uint16_t adcToPulseUs(uint16_t adc)
{
	return (uint16_t)map((long)adc, 0L, (long)ADC_MAX_VALUE, PWM_MIN_US, PWM_MAX_US);
}

void setup()
{
#if !defined(ADC2PWM_PLATFORM_CH32)
	pinMode(PWM_PIN, OUTPUT);
#endif
	pinMode(LED_PIN, OUTPUT);
	ledSet(false);

#if defined(ADC2PWM_PLATFORM_ESP32)
	analogReadResolution(12);
#endif

	bootTimeMs = millis();

	if (esc.attach(PWM_PIN) == INVALID_SERVO) {
		while (true) {
			ledSet(true);
			delay(80);
			ledSet(false);
			delay(80);
		}
	}

	esc.writeMicroseconds(PWM_BOOT_US);
}

void loop()
{
	const unsigned long nowMs = millis();

	if ((nowMs - bootTimeMs) < BOOT_TIME_MS) {
		ledBlink(500, nowMs - bootTimeMs);
		esc.writeMicroseconds(PWM_BOOT_US);
		delay(LOOP_DELAY_MS);
		return;
	}

	if (!safetyArmed) {
		const uint16_t adc = readAdcClamped();
		if (adc <= ADC_ARM_THRESHOLD) {
			safetyArmed = true;
			pwmOutput = PWM_MIN_US;
			pwmTarget = PWM_MIN_US;
			esc.writeMicroseconds(pwmOutput);
			ledSet(true);
			delay(LOOP_DELAY_MS);
			return;
		}

		ledBlink(150, nowMs);
		esc.writeMicroseconds(PWM_BOOT_US);
		delay(LOOP_DELAY_MS);
		return;
	}

	ledSet(true);

	pwmTarget = adcToPulseUs(readAdcClamped());
	if (pwmOutput < pwmTarget) {
		pwmOutput = (uint16_t)(pwmOutput + RAMP_STEP_US_UP);
		if (pwmOutput > pwmTarget) {
			pwmOutput = pwmTarget;
		}
	} else if (pwmOutput > pwmTarget) {
		pwmOutput = (uint16_t)(pwmOutput - RAMP_STEP_US_DOWN);
		if (pwmOutput < pwmTarget) {
			pwmOutput = pwmTarget;
		}
	}

	esc.writeMicroseconds(pwmOutput);
	delay(LOOP_DELAY_MS);
}
