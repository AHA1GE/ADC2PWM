/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00x_it.c
 * Author             : BLDC_MOTOR_FAN_AIO
 * Version            : V1.0.0
 * Date               : 2026-07-18
 * Description        : Interrupt Service Routines.
 *
 * SysTick_Handler      — 1ms system tick
 * TIM1_UP_IRQHandler   — ~16kHz commutation timing reference
 *******************************************************************************/

#include <ch32v00x_it.h>

/*===========================================================================
 * External globals (defined in bldc_hal.c)
 *===========================================================================*/

extern volatile uint32_t sys_tick_millis;
extern volatile uint32_t tim1_tick_count;

/*===========================================================================
 * NMI Handler — Spin
 *===========================================================================*/

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void NMI_Handler(void)
{
    while (1) { }
}

/*===========================================================================
 * HardFault Handler — Reset
 *===========================================================================*/

void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) { }
}

/*===========================================================================
 * SysTick Handler — 1ms system tick
 *
 * Configured in main.c setup(): SysTick->CMP = 6000 (48MHz/8M*1000)
 * HCLK/8 = 6MHz → CMP=6000 → 1ms period.
 *===========================================================================*/

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void SysTick_Handler(void)
{
    SysTick->SR = 0;   // Clear compare flag
    sys_tick_millis++;
}

/*===========================================================================
 * TIM1 Update Handler — Commutation timing reference
 *
 * TIM1 runs at ~15.87kHz (period ~63µs).
 * Just increments a counter; BEMF sampling uses every 4th tick.
 *===========================================================================*/

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        tim1_tick_count++;
    }
}
