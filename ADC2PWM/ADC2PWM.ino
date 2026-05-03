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

/*================================
 * Pin and Hardware Configuration
 *
================================*/

#define USE_OLED_SCREEN 1

#if defined(ADC2PWM_PLATFORM_AVR)
#define PWM_PIN 2
#define ADC_PIN A0
#define BATT_PIN A1
#define LED_PIN 3
#define LED_ACTIVE_HIGH 1  // arduino can push
#define ADC_MAX_VALUE 1023
#if USE_OLED_SCREEN
#define SDA_PIN -1
#define SCL_PIN -1
#endif
#elif defined(ADC2PWM_PLATFORM_CH32)
#define PWM_PIN PD4_TIM2CH1
#define ADC_PIN PA2        // ADC0, potentiometer
#define BATT_PIN PD6       // ADC6, 4.7k/10k divider, 5v vRef -> `vBatt = adc*14.7/4.7*5/ADC_MAX_VALUE`
#define LED_PIN PD1        // onboard LED on CH32V003 EVT boards, active low
#define LED_ACTIVE_HIGH 0  // ch32 use open/drain to sink, active low
#define ADC_MAX_VALUE 1023
#if USE_OLED_SCREEN
#define SDA_PIN PC1
#define SCL_PIN PC2
#endif
#elif defined(ADC2PWM_PLATFORM_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32C3)
#define PWM_PIN -1
#define ADC_PIN -1
#define BATT_PIN -1
#define LED_PIN -1
#define LED_ACTIVE_HIGH -1
#define ADC_MAX_VALUE 4095
#if USE_OLED_SCREEN
#define SDA_PIN -1
#define SCL_PIN -1
#endif
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#define PWM_PIN -1
#define ADC_PIN -1
#define BATT_PIN -1
#define LED_PIN -1
#define LED_ACTIVE_HIGH -1
#define ADC_MAX_VALUE 4095
#if USE_OLED_SCREEN
#define SDA_PIN -1
#define SCL_PIN -1
#endif
#elif !defined(PWM_PIN) || !defined(ADC_PIN) || !defined(LED_PIN) || !defined(LED_ACTIVE_HIGH)
#error "Generic ESP32 requires predefined ADC_PIN, PWM_PIN, LED_PIN and LED_ACTIVE_HIGH before compiling."
#endif
#ifndef ADC_MAX_VALUE
#define ADC_MAX_VALUE 4095
#endif
#endif

/*
 * Compile-time sanity checks
 * Ensure platform pin macros and LED_ACTIVE_HIGH are not left as -1
 * (which indicates "not defined" in some ESP32 configs) because
 * the code calls pinMode/digitalWrite based on these macros.
 */
#if defined(PWM_PIN) && (PWM_PIN == -1)
#error "PWM_PIN is set to -1. Define a valid PWM_PIN for this build target."
#endif
#if defined(ADC_PIN) && (ADC_PIN == -1)
#error "ADC_PIN is set to -1. Define a valid ADC_PIN for this build target."
#endif
#if defined(LED_PIN) && (LED_PIN == -1)
#error "LED_PIN is set to -1. Define a valid LED_PIN for this build target."
#endif
#if defined(LED_ACTIVE_HIGH) && !((LED_ACTIVE_HIGH) == 0 || (LED_ACTIVE_HIGH) == 1)
#error "LED_ACTIVE_HIGH must be 0 or 1 (do not set to -1)."
#endif

/*================================
 * Screen Configuration (Optional)
 *
================================*/

#if USE_OLED_SCREEN && defined(SDA_PIN) && defined(SCL_PIN) && (defined(ADC2PWM_PLATFORM_AVR) || defined(ADC2PWM_PLATFORM_ESP32))
#include <Wire.h>
#include <U8x8lib.h>
#define SCREEN_WIDTH 72                                       // OLED display width, in pixels
#define SCREEN_HEIGHT 40                                      // OLED display height, in pixels
#define SCREEN_RESET -1                                       // OLED display reset pin # (or -1 if sharing Arduino reset pin)
U8X8_SSD1306_72X40_ER_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);  // EastRising 0.42" OLED, 72x40 pixels
#elif USE_OLED_SCREEN && defined(SDA_PIN) && defined(SCL_PIN) && defined(ADC2PWM_PLATFORM_CH32)
/* Note:
 * 1) u8x8 or u8g2 is way too large for ch32v003 flash, use OLED-Basic-Lib instead
 * 2) SDA and SCL pin are hard coded in `OLED_driver.c`. defines above are just for reference and do not have effect on CH32 platform.
 * 3) Usage can be found at (OLED-Basic-Lib)[https://github.com/bdth-7777777/OLED-Basic-Lib/]
*/
#include "OLED.h"
#endif

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

static Servo esc;

static FSM g_fsm = FSM();

State_t stateBoot, stateDisarmed, stateArmed, stateLowBattery, stateError;

static void ledSet(bool on);
static void ledBlink(unsigned long intervalMs, unsigned long nowMs);
static void lowBatteryCheck(void);
static uint16_t readAdcClamped(int16_t pin);
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
		vBattMv = ((uint64_t)readAdcClamped(BATT_PIN) * 147ULL * 5000ULL) / (47ULL * ADC_MAX_VALUE);

		if (vBattMv <= BATTERY_THRESHOLD_1S_MIN) {
			// too low, error
			g_fsm.transition(&stateError);
		} else if (vBattMv <= BATTERY_THRESHOLD_1S_LOW) {
			// 1s low battery, warning
			g_fsm.transition(&stateLowBattery);
		} else if (vBattMv <= BATTERY_THRESHOLD_1S_MAX) {
			// 1s normal, do nothing
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_MIN) {
			// not 1s nor 2s, error
			g_fsm.transition(&stateError);
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_LOW) {
			// 2s low battery, warning
			g_fsm.transition(&stateLowBattery);
		} else if (vBattMv <= BATTERY_THRESHOLD_2S_MAX) {
			// 2s normal, do nothing
		} else {
			// above 2s max voltage, error
			g_fsm.transition(&stateError);
		}
	}
}

static uint16_t readAdcClamped(int16_t PIN) {
	int raw = analogRead(PIN);
	if (raw < 0) {
		raw = 0;
	}
	if (raw > ADC_MAX_VALUE) {
		raw = ADC_MAX_VALUE;
	}
	return (uint16_t)raw;
}

static uint16_t adcToPulseUs(uint16_t adcValue) {
	return (uint16_t)map((long)adcValue, 0L, (long)ADC_MAX_VALUE, PWM_MIN_US, PWM_MAX_US);
}


/*================================
 * OLED Functions
 *
================================*/

#if USE_OLED_SCREEN && (defined(ADC2PWM_PLATFORM_AVR) || defined(ADC2PWM_PLATFORM_ESP32))
static void screenInit(void) {
	u8x8.begin();
	u8x8.setPowerSave(0);
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	u8x8.clear();
	u8x8.drawString(1, 3, "Init...");
	u8x8.refreshDisplay();
}
static void screenClear(void) {
	u8x8.clear();
	u8x8.refreshDisplay();
}
static void screenShowDisarmed(void) {
	u8x8.clear();
	u8x8.drawString(0, 3, "DISARMED");
	u8x8.refreshDisplay();
}
static void screenPrintPrepare(void) {
	u8x8.clear();
	u8x8.drawString(0, 1, "Bat:");
	u8x8.drawString(0, 3, "Thr:");
	u8x8.drawString(0, 5, "PWM:");
	u8x8.refreshDisplay();
}
static void screenPrint(void) {
	char buffer[16];
	// format vBattMv to `x.xV` with 1 decimal place
	snprintf(buffer, sizeof(buffer), "%lu.%luV", (unsigned long)(vBattMv / 1000), (unsigned long)((vBattMv % 1000) / 100));
	// `Bat:` length 4, +1 for spacing
	u8x8.drawString(5, 1, buffer);
	snprintf(buffer, sizeof(buffer), "%u", throttleAdc);
	// `Thr:` length 4, +1 for spacing
	u8x8.drawString(5, 3, buffer);
	snprintf(buffer, sizeof(buffer), "%u", pwmOutput);
	// `PWM:` length 4, +1 for spacing
	u8x8.drawString(5, 5, buffer);
	u8x8.refreshDisplay();
}
static void screenLowBatteryWarning(void) {
	// blink low battery warning at the right corner
	if (millis() % 1000 < 500) {
		u8x8.drawString(8, 1, "!");
	} else {
		u8x8.drawString(8, 1, " ");
	}
	screenPrint();
}
#elif USE_OLED_SCREEN && defined(ADC2PWM_PLATFORM_CH32)
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
#endif

/*================================
 * State Implementations
 *
================================*/
void stateBootEnter() {
	bootTimeMs = millis();

	if (esc.attach(PWM_PIN) == INVALID_SERVO) {
		g_fsm.transition(&stateError);
	}

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

	throttleAdc = readAdcClamped(ADC_PIN);
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
	throttleAdc = readAdcClamped(ADC_PIN);
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
	throttleAdc = readAdcClamped(ADC_PIN);
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
	stateArmed.stateExit = nullptr;

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
#if !defined(ADC2PWM_PLATFORM_CH32)
	pinMode(PWM_PIN, OUTPUT);  // CH32 has dedicated pin setup
#endif
	pinMode(ADC_PIN, INPUT);
	pinMode(LED_PIN, OUTPUT);
	ledSet(false);
	if (BATT_PIN != -1) {
		pinMode(BATT_PIN, INPUT);
	}

#if defined(ADC2PWM_PLATFORM_ESP32)
	analogReadResolution(12);
#endif

	g_fsm.init(&stateBoot);
}

void loop() {
	g_fsm.run();
	delay(10);
}
