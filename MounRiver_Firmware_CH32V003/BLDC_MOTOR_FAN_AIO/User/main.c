/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : BLDC_MOTOR_FAN_AIO
 * Version            : V1.0.0
 * Date               : 2026-07-18
 * Description        : BLDC Fan Controller — CH32V003F4P6 + DRV8311 + CH1115 OLED
 *
 * BEMF sensorless 6-step commutation, duty via tSPI, OLED status display,
 * 2-button throttle control.
 *******************************************************************************/

#include "debug.h"
#include "ch32v00x.h"
#include "bldc_hal.h"
#include "bldc_control.h"
#include "OLED.h"
#include "OLED_driver.h"
#include <stdio.h>
#include <string.h>

/*===========================================================================
 * Global State
 *===========================================================================*/

static uint32_t last_btn_millis;
static uint32_t last_fault_millis;
static uint32_t last_batt_millis;
static uint32_t last_oled_millis;
static uint32_t last_current_millis;
static uint32_t fault_blink_millis;
static uint8_t  fault_blink_state;

static uint16_t cached_current_mA;

/*===========================================================================
 * OLED Display Update — 3 lines: BAT, DUT%, CUR
 *===========================================================================*/

static void display_update(void)
{
    char buf[22];

    OLED_Clear();

    // Line 1 (y=0): Battery voltage
    uint16_t batt_mv = BAT_Get_mV();
    sprintf(buf, "BAT:%d.%02dV", batt_mv / 1000, (batt_mv % 1000) / 10);
    OLED_ShowMixStringArea(0, 0, OLED_WIDTH, 8, 0, 0, buf, OLED_FONT_8);

    // Line 2 (y=20): Throttle / Duty percentage
    if (motor.state == STATE_FAULT) {
        sprintf(buf, "DUT:FAULT");
    } else {
        sprintf(buf, "DUT:%3d%%", motor.throttle_pct);
    }
    OLED_ShowMixStringArea(0, 20, OLED_WIDTH, 8, 0, 20, buf, OLED_FONT_8);

    // Line 3 (y=40): Current or state
    if (motor.state == STATE_FAULT) {
        sprintf(buf, "CUR:FAULT!");
    } else if (motor.state == STATE_IDLE) {
        sprintf(buf, "CUR:IDLE");
    } else {
        sprintf(buf, "CUR:%d.%02dA",
                cached_current_mA / 1000, (cached_current_mA % 1000) / 10);
    }
    OLED_ShowMixStringArea(0, 40, OLED_WIDTH, 8, 0, 40, buf, OLED_FONT_8);

    OLED_Update();
}

/*===========================================================================
 * Fault LED Blink — 5Hz on PD1
 *===========================================================================*/

static void fault_blink_poll(void)
{
    uint32_t now = sys_tick_millis;
    if (now - fault_blink_millis >= 100) {  // 100ms period = 5Hz
        fault_blink_millis = now;
        fault_blink_state = !fault_blink_state;
        if (fault_blink_state)
            LED_On();
        else
            LED_Off();
    }
}

/*===========================================================================
 * Fault Monitoring
 *===========================================================================*/

static uint8_t nfault_consecutive;

static void fault_check(void)
{
    // nFAULT is open-drain from DRV8311, pulled up externally.
    // Low = fault asserted.
    if (GPIO_ReadInputDataBit(GPIOD, PIN_nFAULT) == Bit_RESET) {
        nfault_consecutive++;
        if (nfault_consecutive >= 3 && motor.state != STATE_FAULT) {
            BLDC_EnterFault();
        }
    } else {
        nfault_consecutive = 0;
    }

    // Battery critical check
    uint16_t batt_mv = BAT_Get_mV();
    if (batt_mv > 0 && batt_mv < BATT_CRITICAL_MV && motor.state != STATE_IDLE) {
        if (motor.state != STATE_FAULT) {
            BLDC_EnterFault();
        }
    }
}

/*===========================================================================
 * Boot LED Blink
 *===========================================================================*/

static void boot_blink(void)
{
    LED_On();
    Delay_Ms(200);
    LED_Off();
    Delay_Ms(200);
}

/*===========================================================================
 * Splash Screen
 *===========================================================================*/

static void show_splash(void)
{
    OLED_Clear();
    OLED_ShowMixStringArea(0, 10, OLED_WIDTH, 8, 0, 10, "BLDC FAN", OLED_FONT_8);
    OLED_ShowMixStringArea(0, 30, OLED_WIDTH, 8, 0, 30, "AIO v1.0", OLED_FONT_8);
    OLED_Update();
    Delay_Ms(1000);
}

/*===========================================================================
 * DRV8311 Init + Diagnostics
 *===========================================================================*/

static int drv8311_setup(void)
{
    int ret = DRV8311_Init();

    if (ret != 0) {
        // Init failed at step 'ret' — display error
        OLED_Clear();
        char buf[22];
        sprintf(buf, "DRV ERR %d", ret);
        OLED_ShowMixStringArea(0, 20, OLED_WIDTH, 8, 0, 20, buf, OLED_FONT_8);
        OLED_Update();

        // Blink LED rapidly to indicate error
        while (1) {
            LED_Toggle();
            Delay_Ms(100);
        }
    }

    // Read DEV_STS1 to confirm communication
    uint16_t sts1 = drv8311_read_reg(drv, DRV8311_DEV_STS1_ADDR);
    (void)sts1;  // Could display on OLED if desired

    // Enable PWM output
    DRV8311_Enable();

    return 0;
}

/*===========================================================================
 * Setup — Blocking Initialization
 *===========================================================================*/

static void setup(void)
{
    // Clock + delay
    SystemCoreClockUpdate();
    Delay_Init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

    // Hardware init
    HAL_GPIO_Init();       // All 20 pins per PIN.md
    HAL_SPI_Init();        // SPI1 master, 6MHz, CS=PC3 GPIO
    HAL_ADC_Init();        // 8 analog channels, single-channel, software trigger
    HAL_TIM1_Init();       // 15.87kHz commutation timing reference

    // OLED
    OLED_Init();
    OLED_Clear();
    OLED_Update();

    // Boot blink (before DRV8311 init, since LED is on PD1)
    // LED not configured yet — skip for now, will show on OLED

    // DRV8311
    drv8311_setup();

    // Splash screen
    show_splash();

    // SWD connection window (PD1 is still SWDIO at this point)
    Delay_Ms(100);

    // Configure LED (PD1) — LAST, after SWD window
    LED_Init();
    boot_blink();

    // Start system tick (1ms) — must be after last Delay_Ms() call
    SysTick->CMP = SystemCoreClock / 8000000 * 1000;  // 1ms at 48MHz
    SysTick->CNT = 0;
    SysTick->CTLR = 0x7;  // Enable, interrupt, HCLK
    // SysTick interrupt is already enabled in startup via NVIC

    // Initialize motor control
    BLDC_Init();

    // Optional debug via SDI (WCH-Link)
    // SDI_Printf_Enable();

    // Force an initial battery reading so display shows something
    BAT_ADC_Sample();
    cached_current_mA = 0;
}

/*===========================================================================
 * Main Loop — Non-blocking, SysTick-driven
 *===========================================================================*/

int main(void)
{
    setup();

    while (1) {
        uint32_t now = sys_tick_millis;

        /* --- Button processing: every 10ms --- */
        if (now - last_btn_millis >= BTN_POLL_MILLIS) {
            last_btn_millis = now;
            BTN_Process();
            BLDC_ProcessButtons();
        }

        /* --- Fault check: every 50ms --- */
        if (now - last_fault_millis >= FAULT_CHECK_MILLIS) {
            last_fault_millis = now;
            fault_check();
        }

        /* --- Battery ADC: every 500ms --- */
        if (now - last_batt_millis >= BATT_ADC_MILLIS) {
            last_batt_millis = now;
            BAT_ADC_Sample();
        }

        /* --- Current sampling: every 200ms (when running) --- */
        if (motor.state == STATE_RUN &&
            now - last_current_millis >= 200) {
            last_current_millis = now;
            cached_current_mA = BLDC_GetBusCurrent_mA();
        }

        /* --- State machine: run every iteration --- */
        if (motor.state == STATE_FAULT) {
            fault_blink_poll();
        } else {
            LED_Off();
            BLDC_StateMachine();
        }

        /* --- OLED update: every 100ms --- */
        if (now - last_oled_millis >= OLED_UPDATE_MILLIS) {
            last_oled_millis = now;
            display_update();
        }
    }
}
