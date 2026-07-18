#ifndef __BLDC_HAL_H
#define __BLDC_HAL_H

#include <ch32v00x.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * GPIO Pin Definitions (per PIN.md — single source of truth)
 *===========================================================================*/

// Digital I/O
#define PIN_nFAULT          GPIO_Pin_7   // PD7 — DRV8311 nFAULT input
#define PIN_BTN_UP          GPIO_Pin_0   // PD0 — Button Up (active low, IPU)
#define PIN_BTN_DOWN        GPIO_Pin_0   // PC0 — Button Down (active low, IPU)
#define PIN_LED             GPIO_Pin_1   // PD1 — LED (shared SWDIO, init last)
#define PIN_SPI_CS          GPIO_Pin_3   // PC3 — DRV8311 SPI CS (manual GPIO)

// Analog inputs — PIN.md A0-A7 = ADC_Channel_0..7 directly
#define ADC_CH_BATT             ADC_Channel_0   // PA2 (A0) — battery divider
#define ADC_CH_BEMF_NEUTRAL     ADC_Channel_1   // PA1 (A1) — virtual neutral
#define ADC_CH_CURR_U           ADC_Channel_2   // PC4 (A2) — DRV8311 ISEN_U
#define ADC_CH_CURR_V           ADC_Channel_3   // PD2 (A3) — DRV8311 ISEN_V
#define ADC_CH_CURR_W           ADC_Channel_4   // PD3 (A4) — DRV8311 ISEN_W
#define ADC_CH_BEMF_U           ADC_Channel_5   // PD5 (A5) — BEMF phase U
#define ADC_CH_BEMF_V           ADC_Channel_6   // PD6 (A6) — BEMF phase V
#define ADC_CH_BEMF_W           ADC_Channel_7   // PD4 (A7) — BEMF phase W

/*===========================================================================
 * DRV8311 Configuration
 *===========================================================================*/

// PWM
#define DRV_PWM_PERIOD          960     // ~25kHz with DRV8311 internal ~24MHz osc
#define DRV_PWM_ALIGN_DUTY      144     // ~15% of 960 for rotor alignment
#define DRV_PWM_MIN_DUTY        144     // Minimum running duty (~15%)
#define DRV_PWM_MAX_DUTY        960     // Maximum = period (100%)

// Battery thresholds (mV)
#define BATT_CRITICAL_MV        3000
#define BATT_LOW_MV             3300
#define BATT_OK_MAX_MV          4500

// Battery divider: R4=10K (high), R5=4.7K (low)
// V_ADC = V_BAT * 4.7 / (10 + 4.7) = V_BAT * 0.3197
// V_BAT = V_ADC / 0.3197 = V_ADC * 3.127
// ADC = V_ADC * 1024 / 3.3
// V_BAT_mV = ADC * 3300 / 1024 / 0.3197 = ADC * 10.09
// Simplified: batt_mv = (adc_val * 3300UL / 1024) * 147 / 47

// Timing
#define FAULT_CHECK_MILLIS      50
#define BATT_ADC_MILLIS         500
#define OLED_UPDATE_MILLIS      100
#define BTN_POLL_MILLIS         10

/*===========================================================================
 * Public API
 *===========================================================================*/

void HAL_GPIO_Init(void);
void HAL_SPI_Init(void);
void HAL_ADC_Init(void);
void HAL_TIM1_Init(void);
void HAL_TIM1_Start(void);
void HAL_TIM1_Stop(void);

// LED (PD1, shared SWDIO — init last)
void LED_Init(void);

// DRV8311
#include "drv8311.h"
extern drv8311_handle_t drv;

int  DRV8311_Init(void);
void DRV8311_Enable(void);
void DRV8311_Disable(void);

// SPI callback for drv8311.c (hardware SPI1)
void spi_trans_cb(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t rx_len);

// nSLEEP no-op (hardwired to BAT+ on this board)
void nsleep_set_cb(uint8_t level);

// Battery
void BAT_ADC_Sample(void);
uint16_t BAT_Get_mV(void);

// Buttons (debounced, call at ~10ms intervals)
bool BTN_Up_Pressed(void);
bool BTN_Down_Pressed(void);
bool BTN_Up_IsLongPress(void);
bool BTN_Down_IsLongPress(void);
void BTN_Process(void);

// LED
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

// System tick (from SysTick ISR)
extern volatile uint32_t sys_tick_millis;
extern volatile uint32_t tim1_tick_count;

#ifdef __cplusplus
}
#endif

#endif /* __BLDC_HAL_H */
