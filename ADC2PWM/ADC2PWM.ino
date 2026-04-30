/*
 * Unified ADC2PWM firmware for AVR, CH32 and ESP32.
 *
 * Behavior:
 * 1) Output PWM_BOOT_US during boot window with slow LED blink.
 * 2) Require ADC minimum once to unlock normal control.
 * 3) After unlock, map ADC to PWM range with asymmetric ramp limiting.
 */

#include "Servo.h"
#include "Fsm.h"

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
#define PWM_PIN PC4_TIM1CH4
#define ADC_PIN PA2		  // ADC0, potentiometer
#define BATT_PIN PD6	  // ADC6, 4.7k/10k divider, 5v vRef -> `vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE`
#define LED_PIN PD4		  // same physical pin as SWD on CH32V003J4M6
#define LED_ACTIVE_HIGH 0 // ch32 use open/drain to sink, active low
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
#define LOW_BATTERY_MILLIVOLT_THRESHOLD_1S 3300 // 3.3V (for 1s battery)
#define LOW_BATTERY_MILLIVOLT_THRESHOLD_2S 6500 // 6.5V (for 2s battery)

static uint16_t pwmOutput = PWM_BOOT_US;
static uint16_t pwmTarget = PWM_BOOT_US;
static unsigned long bootTimeMs = 0;
static bool safetyArmed = false;

static Servo esc;

static FSM g_fsm = FSM();

State_t stateBoot, stateDisarmed, stateArmed, stateLowBattery, stateError;

static void ledSet(bool on)
{
	const uint8_t level = LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
	digitalWrite(LED_PIN, level);
}

static void ledBlink(unsigned long intervalMs, unsigned long nowMs)
{
	ledSet(((nowMs / intervalMs) % 2u) == 0u);
}

static void lowBatteryCheck()
{
	if (BATT_PIN != NULL)
	{
		const uint16_t adc = readAdcClamped(BATT_PIN);
		// 1s-2s battery with 4.7k/10k divider -> vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE
		const uint32_t vBattMv = (uint32_t)adc * 147 * 5 / 47 / ADC_MAX_VALUE;
		// decide
		if (vBattMv < LOW_BATTERY_MILLIVOLT_THRESHOLD_1S)
		{
			// lower than 1s threshold
			g_fsm.transition(&stateLowBattery);
		}
		else if (4400 < vBattMv && vBattMv < 6000){
			// between 1s and 2s, error
			g_fsm.transition(&stateError);
		}
		else if (vBattMv < LOW_BATTERY_MILLIVOLT_THRESHOLD_2S)
		{
			// lower than 2s threshold
			g_fsm.transition(&stateLowBattery);
		}
		else{
			// higher than 2s threshold, do nothing
		}
	}
}

static uint16_t readAdcClamped(uint8_t PIN)
{
	int raw = analogRead(PIN);
	if (raw < 0)
	{
		raw = 0;
	}
	if (raw > ADC_MAX_VALUE)
	{
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
	delay(100);
#if !defined(ADC2PWM_PLATFORM_CH32)
	pinMode(PWM_PIN, OUTPUT); // CH32 has dedicated pin setup
#endif
    pinMode(ADC_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);
	ledSet(false);
	if (BATT_PIN != NULL)
	{
		pinMode(BATT_PIN, INPUT);
	}

#if defined(ADC2PWM_PLATFORM_ESP32)
	analogReadResolution(12);
#endif

	g_fsm.init(&stateBoot);
}

void loop()
{
	g_fsm.run();
	delay(10);
}

void stateBootEnter()
{
	bootTimeMs = millis();

	if (esc.attach(PWM_PIN) == INVALID_SERVO)
	{
		// Fast blink to indicate ERROR
		while (true)
		{
			ledSet(true);
			delay(80);
			ledSet(false);
			delay(80);
		}
	}

	esc.writeMicroseconds(PWM_BOOT_US);
}
void stateBootRun()
{
	const unsigned long nowMs = millis();
	if ((nowMs - bootTimeMs) >= BOOT_TIME_MS)
	{
		g_fsm.transition(&stateDisarmed);
	}
}
void stateDisarmedEnter()
{
	ledSet(false);
	pwmOutput = PWM_BOOT_US;
	pwmTarget = PWM_BOOT_US;
	esc.writeMicroseconds(pwmOutput);
}
void stateDisarmedRun()
{
	const unsigned long nowMs = millis();

	const uint16_t adc = readAdcClamped(ADC_PIN);
	if (adc <= ADC_ARM_THRESHOLD)
	{
		pwmOutput = PWM_MIN_US;
		pwmTarget = PWM_MIN_US;
		esc.writeMicroseconds(pwmOutput);
		ledSet(true);
		g_fsm.transition(&stateArmed);
	}
	else
	{
		ledBlink(150, nowMs);
		esc.writeMicroseconds(PWM_BOOT_US);
	}
}
void stateArmedEnter()
{
	ledSet(true);
	pwmOutput = PWM_MIN_US;
	pwmTarget = PWM_MIN_US;
	esc.writeMicroseconds(pwmOutput);
}
void stateArmedRun()
{
	lowBatteryCheck();
	const uint16_t adc = readAdcClamped(ADC_PIN);
	pwmTarget = adcToPulseUs(adc);
	if (pwmOutput < pwmTarget)
	{
		pwmOutput = (uint16_t)(pwmOutput + RAMP_STEP_US_UP);
		if (pwmOutput > pwmTarget)
		{
			pwmOutput = pwmTarget;
		}
	}
	else if (pwmOutput > pwmTarget)
	{
		pwmOutput = (uint16_t)(pwmOutput - RAMP_STEP_US_DOWN);
		if (pwmOutput < pwmTarget)
		{
			pwmOutput = pwmTarget;
		}
	}
	esc.writeMicroseconds(pwmOutput);
}
void stateArmedExit()
{
	ledSet(false);
	pwmOutput = PWM_BOOT_US;
	pwmTarget = PWM_BOOT_US;
	esc.writeMicroseconds(pwmOutput);
}
void stateLowBatteryRun()
{
	// scale throttle down to 50% and blink LED
	const uint16_t adc = readAdcClamped(ADC_PIN);
	const unsigned long nowMs = millis();
	pwmTarget = (uint16_t)(adcToPulseUs(adc * 0.5));
	if (pwmOutput < pwmTarget)
	{
		pwmOutput = (uint16_t)(pwmOutput + RAMP_STEP_US_UP);
		if (pwmOutput > pwmTarget)
		{
			pwmOutput = pwmTarget;
		}
	}
	else if (pwmOutput > pwmTarget)
	{
		pwmOutput = (uint16_t)(pwmOutput - RAMP_STEP_US_DOWN);
		if (pwmOutput < pwmTarget)
		{
			pwmOutput = pwmTarget;
		}
	}
	esc.writeMicroseconds(pwmOutput);
	ledBlink(300, nowMs);
}
void stateErrorRun()
{
	// Lock the motor
	esc.writeMicroseconds(PWM_BOOT_US);
	// Fast blink to indicate ERROR
	while (true)
	{
		ledSet(true);
		delay(80);
		ledSet(false);
		delay(80);
	}
}

State_t stateBoot = {
	.stateName = "stateBoot",
	.stateEnter = stateBootEnter,
	.stateRun = stateBootRun,
	.stateExit = nullptr};
State_t stateDisarmed = {
	.stateName = "stateDisarmed",
	.stateEnter = stateDisarmedEnter,
	.stateRun = stateDisarmedRun,
	.stateExit = nullptr};
State_t stateArmed = {
	.stateName = "stateArmed",
	.stateEnter = stateArmedEnter,
	.stateRun = stateArmedRun,
	.stateExit = nullptr};
State_t stateLowBattery = {
	.stateName = "stateLowBattery",
	.stateEnter = nullptr,
	.stateRun = stateLowBatteryRun,
	.stateExit = nullptr};
State_t stateError = {
	.stateName = "stateError",
	.stateEnter = nullptr,
	.stateRun = stateErrorRun,
	.stateExit = nullptr};