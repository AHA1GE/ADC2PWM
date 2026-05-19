/*
 * Unified ADC2PWM firmware for AVR, CH32 and ESP32.
 *
 * Behavior:
 * 1) Output PWM_BOOT_US during boot window with slow LED blink.
 * 2) Require ADC minimum once to unlock normal control.
 * 3) After unlock, map ADC to PWM range with asymmetric ramp limiting.
 */

#include "Fsm.h"
#include "OLED.h"
#include "CH32V003_SERVO.h"

/*================================
 * Pin and Hardware Configuration
 *
================================*/

#define PWM_PIN SERVO_PIN_PC4_TIM1_CH4
#define ADC_PIN PA2        // ADC0, potentiometer
#define BATT_PIN PD6       // ADC6, 4.7k/10k divider, 5v vRef -> `vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE`
#define LED_PIN PD4        // same physical pin as SWD on CH32V003J4M6
#define LED_ACTIVE_HIGH 0  // ch32 use open/drain to sink, active low
#define ADC_MAX_VALUE 1023
#define SDA_PIN PC1
#define SCL_PIN PC2

/*================================
 * PWM, Timing and Safety Configurations
 *
================================*/

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
#define BATTERY_THRESHOLD_1S_MIN 3000  // 3.0V (for 1s battery, critical low voltage)
#define BATTERY_THRESHOLD_1S_LOW 3300  // 3.3V (for 1s battery)
#define BATTERY_THRESHOLD_1S_MAX 4350  // 4.35V (for 1s battery, 4.2V full charge + 0.15V margin)
#define BATTERY_THRESHOLD_2S_MIN 6000  // 6.0V (for 2s battery, critical low voltage)
#define BATTERY_THRESHOLD_2S_LOW 6500  // 6.5V (for 2s battery)
#define BATTERY_THRESHOLD_2S_MAX 8700  // 8.7V (for 2s battery, 8.4V full charge + 0.3V margin)

/*================================
 * Global Variables and State Definitions
 *
================================*/

static uint16_t pwmOutput = PWM_BOOT_US;
static uint16_t pwmTarget = PWM_BOOT_US;
static unsigned long bootTimeMs = 0;
static uint32_t vBattMv = 0;
static uint16_t throttleAdc = 0;
static char errorMessageBuffer[10] = {0};  // buffer for error message, max 9 chars + null terminator

static Servo esc;

static FSM g_fsm = FSM();

State_t stateBoot, stateDisarmed, stateArmed, stateLowBattery, stateError;

static void ledSet(bool on);
static void ledBlink(unsigned long intervalMs, unsigned long nowMs);
static void lowBatteryCheck(void);
static uint16_t adcToPulseUs(uint16_t adcValue);
static void screenInit(void);
static void screenClear(void);
static void screenShowDisarmed(void);
static void screenPrintPrepare(void);
static void screenPrint(void);
static void screenLowBatteryWarning(void);
static void stateBootEnter(void);
static void stateBootRun(void);
static void stateBootExit(void);
static void stateDisarmedEnter(void);
static void stateDisarmedRun(void);
static void stateArmedEnter(void);
static void stateArmedRun(void);
static void stateArmedExit(void);
static void stateLowBatteryRun(void);
static void stateErrorRun(void);

/*================================
 * Helper Functions
 *
================================*/

static void ledSet(bool on) {
	const uint8_t level = LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH);
	digitalWrite(LED_PIN, level);
}

static void ledBlink(unsigned long intervalMs, unsigned long nowMs) {
	ledSet(((nowMs / intervalMs) % 2u) == 0u);
}

static void lowBatteryCheck() {
	if (BATT_PIN != -1) {
		// 1s-2s battery, divider: gnd_4.7k_battAdc_10k_vBatt, ref = 5000mV
		vBattMv = ((uint64_t)analogRead(BATT_PIN) * 147ULL * 5000ULL) / (47ULL * ADC_MAX_VALUE);

		if (vBattMv <= BATTERY_THRESHOLD_1S_MIN) {
			// too low, error
			snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "BAT LOW!");
			g_fsm.transition(&stateError);
		} else if (vBattMv <= BATTERY_THRESHOLD_1S_LOW) {
			// 1s low battery, warning
			g_fsm.transition(&stateLowBattery);
		} else if (vBattMv <= BATTERY_THRESHOLD_1S_MAX) {
			// 1s normal, do nothing
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_MIN) {
			// not 1s nor 2s, error
			snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "BAT ERR!");
			g_fsm.transition(&stateError);
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_LOW) {
			// 2s low battery, warning
			g_fsm.transition(&stateLowBattery);
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_MAX) {
			// 2s normal, do nothing
		} else {
			// above 2s max voltage, error
			snprintf(errorMessageBuffer, sizeof(errorMessageBuffer), "BAT HIGH!");
			g_fsm.transition(&stateError);
		}
	}
}

static uint16_t adcToPulseUs(uint16_t adcValue) {
	return (uint16_t)map((long)adcValue, 0L, (long)ADC_MAX_VALUE, PWM_MIN_US, PWM_MAX_US);
}


/*================================
 * OLED Functions
 *
================================*/

static void screenInit(void) {
	OLED_Init();
	OLED_SetBrightness(50);
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 9, 25, "Init...", OLED_FONT_8);  //OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenClear(void) {
	OLED_Clear();
	OLED_Update();
}
static void screenShowDisarmed(void) {
	OLED_Clear();
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 25, "DISARMED", OLED_FONT_8);  //OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenPrintPrepare(void) {
	OLED_Clear();
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 0, "Bat:", OLED_FONT_8);   //OLED显示字符数组（字符串）
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 16, "Thr:", OLED_FONT_8);  //OLED显示字符数组（字符串）
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 0, 32, "PWM:", OLED_FONT_8);  //OLED显示字符数组（字符串）
	OLED_Update();
}
static void screenPrint(void) {
	char buffer[16];

	// format vBattMv to `x.xV` with 1 decimal place
	snprintf(buffer, sizeof(buffer), "%lu.%luV", (unsigned long)(vBattMv / 1000), (unsigned long)((vBattMv % 1000) / 100));
	// `Bat:` length 4, +1 for spacing
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 0, buffer, OLED_FONT_8);  //OLED显示字符数组（字符串）

	snprintf(buffer, sizeof(buffer), "%u", throttleAdc);
	// `Thr:` length 4, +1 for spacing
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 16, buffer, OLED_FONT_8);  //OLED显示字符数组（字符串）

	snprintf(buffer, sizeof(buffer), "%u", pwmOutput);
	// `PWM:` length 4, +1 for spacing
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 32, 32, buffer, OLED_FONT_8);  //OLED显示字符数组（字符串）

	OLED_Update();
}
static void screenLowBatteryWarning(void) {
	// blink low battery warning at the right corner
	if (millis() % 1000 < 500) {
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 64, 0, "!", OLED_FONT_8);  //OLED显示字符数组（字符串）
	} else {
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 64, 0, " ", OLED_FONT_8);  //OLED显示字符数组（字符串）
	}
	screenPrint();
}
static void screenShowError( const char* message, uint8_t messageLength) {
	// chack length, no more than 9 characters
	if (messageLength > 9) {
		// do nothing
	}else{
		OLED_Clear();
	    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 16, "ERROR", OLED_FONT_8);  //OLED显示字符数组（字符串）
		OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, 16, 32, message, OLED_FONT_8);  //OLED显示字符数组（字符串）
	    OLED_Update();
	}
}

/*================================
 * State Implementations
 *
================================*/
void stateBootEnter() {
	bootTimeMs = millis();
	// attach servo (CH32 implementation of Servo::attach is void)
	esc.attach(PWM_PIN);

	esc.writeMicroseconds(PWM_BOOT_US);
}
void stateBootRun() {
	const unsigned long nowMs = millis();
	if ((nowMs - bootTimeMs) >= BOOT_TIME_MS) {
		g_fsm.transition(&stateDisarmed);
	}
}
void stateBootExit() {
	screenClear();
}
void stateDisarmedEnter() {
	ledSet(false);
	pwmOutput = PWM_BOOT_US;
	pwmTarget = PWM_BOOT_US;
	esc.writeMicroseconds(pwmOutput);
	screenShowDisarmed();
}
void stateDisarmedRun() {
	const unsigned long nowMs = millis();

	throttleAdc = analogRead(ADC_PIN);
	if (throttleAdc <= ADC_ARM_THRESHOLD) {
		pwmOutput = PWM_MIN_US;
		pwmTarget = PWM_MIN_US;
		esc.writeMicroseconds(pwmOutput);
		ledSet(true);
		g_fsm.transition(&stateArmed);
	} else {
		ledBlink(150, nowMs);
		esc.writeMicroseconds(PWM_BOOT_US);
	}
}
void stateArmedEnter() {
	ledSet(true);
	pwmOutput = PWM_MIN_US;
	pwmTarget = PWM_MIN_US;
	esc.writeMicroseconds(pwmOutput);
	screenPrintPrepare();
}
void stateArmedRun() {
	lowBatteryCheck();
	throttleAdc = analogRead(ADC_PIN);
	pwmTarget = adcToPulseUs(throttleAdc);
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

	screenPrint();
}
void stateArmedExit() {
	ledSet(false);
	pwmOutput = PWM_BOOT_US;
	pwmTarget = PWM_BOOT_US;
	esc.writeMicroseconds(pwmOutput);
}
void stateLowBatteryRun() {
	// scale throttle down to 50% and blink LED
	throttleAdc = analogRead(ADC_PIN);
	const unsigned long nowMs = millis();
	pwmTarget = (uint16_t)(adcToPulseUs(throttleAdc / 2));
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
	ledBlink(300, nowMs);

	screenLowBatteryWarning();
}
void stateErrorRun() {
	// Lock the motor
	esc.writeMicroseconds(PWM_BOOT_US);
	// Fast blink to indicate ERROR
	ledBlink(80, millis());
	// show error message if screen is available
	screenShowError(errorMessageBuffer, strlen(errorMessageBuffer));
}

/*================================
 * Arduino Setup and Loop
 *
================================*/

void setup() {
	stateBoot.stateName = "stateBoot";
	stateBoot.stateEnter = stateBootEnter;
	stateBoot.stateRun = stateBootRun;
	stateBoot.stateExit = stateBootExit;

	stateDisarmed.stateName = "stateDisarmed";
	stateDisarmed.stateEnter = stateDisarmedEnter;
	stateDisarmed.stateRun = stateDisarmedRun;
	stateDisarmed.stateExit = nullptr;

	stateArmed.stateName = "stateArmed";
	stateArmed.stateEnter = stateArmedEnter;
	stateArmed.stateRun = stateArmedRun;
	stateArmed.stateExit = stateArmedExit;

	stateLowBattery.stateName = "stateLowBattery";
	stateLowBattery.stateEnter = nullptr;
	stateLowBattery.stateRun = stateLowBatteryRun;
	stateLowBattery.stateExit = nullptr;

	stateError.stateName = "stateError";
	stateError.stateEnter = nullptr;
	stateError.stateRun = stateErrorRun;
	stateError.stateExit = nullptr;
	
	screenInit();

	delay(100);

	pinMode(PWM_PIN, OUTPUT);  // CH32 has dedicated pin setup

	pinMode(ADC_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);
	ledSet(false);
	if (BATT_PIN != -1) {
		pinMode(BATT_PIN, INPUT);
	}

	g_fsm.init(&stateBoot);
}

void loop() {
	g_fsm.run();
	delay(10);
}
