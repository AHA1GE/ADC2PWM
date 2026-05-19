/*
 * CH32V003_SERVO.c
 *
 * TIM2 PWM servo output for CH32V003 (48 MHz system clock).
 *
 * Channel mapping (default, no AFIO remap required):
 *   TIM2_CH1 → PD4   (SERVO_PIN_PD4_TIM2_CH1)
 *   TIM2_CH4 → PD7   (SERVO_PIN_PD7_TIM2_CH4)
 *   TIM1_CH4 → PC4   (SERVO_PIN_PC4_TIM1_CH4)
 *
 * Timer settings:
 *   PSC = 47  → 48 MHz / 48 = 1 MHz → 1 us per tick
 *   ARR = 19999 → 20 000 us = 20 ms period → 50 Hz
 *   CCR range [500, 2500] covers standard servo / ESC pulse widths.
 *
 * Per RM §11.3.5: ARR and CCR preload registers are enabled; a software
 * update event (UG bit) is generated before the counter starts so that
 * all shadow registers are loaded with their preload values.
 */

#include "CH32V003_SERVO.h"
#include <Arduino.h>

/* -------------------------------------------------------------------------- */
/* Constants                                                                   */
/* -------------------------------------------------------------------------- */

#define SERVO_PSC      47u      /* prescaler: 48 MHz / 48 = 1 MHz (1 us/tick) */
#define SERVO_ARR      19999u   /* auto-reload: 20 000 ticks = 20 ms, 50 Hz   */
#define SERVO_DEFAULT  1500u    /* neutral pulse: 1500 us                      */
#define SERVO_MIN_US   500u     /* minimum allowed pulse width                 */
#define SERVO_MAX_US   2500u    /* maximum allowed pulse width                 */

/* -------------------------------------------------------------------------- */
/* Internal state                                                              */
/* -------------------------------------------------------------------------- */

static uint8_t s_initialized      = 0;
static uint8_t s_tim1_initialized = 0;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static uint16_t clamp_us(uint16_t us)
{
    if (us < SERVO_MIN_US) return (uint16_t)SERVO_MIN_US;
    if (us > SERVO_MAX_US) return (uint16_t)SERVO_MAX_US;
    return us;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void CH32V003_SERVO_Init(void)
{
    if (s_initialized) return;
    s_initialized = 1;

    /* ----- Enable peripheral clocks ----- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,  ENABLE);

    /* ----- GPIO: PD4 (TIM2_CH1) and PD7 (TIM2_CH4) as AF push-pull ----- */
    /* Default pin mapping, no AFIO remap required.                         */
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_7;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpio);

    /* ----- TIM2 time-base: 50 Hz, 1 us resolution ----- */
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = SERVO_ARR;
    tb.TIM_Prescaler     = SERVO_PSC;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tb);

    /* ----- CH1 (PD4): PWM mode 1, edge-aligned ----- */
    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = SERVO_DEFAULT;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* ----- CH4 (PD7): PWM mode 1, edge-aligned ----- */
    oc.TIM_Pulse = SERVO_DEFAULT;
    TIM_OC4Init(TIM2, &oc);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    /* Enable ARR preload (RM §11.3.5: set ARPE) */
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    /* Generate software update event to transfer preload → shadow registers
     * before the counter starts (RM §11.3.5: "置UG位来初始化所有寄存器"). */
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);

    TIM_Cmd(TIM2, ENABLE);
}

void CH32V003_SERVO_WriteCH1(uint16_t us)
{
    TIM_SetCompare1(TIM2, clamp_us(us));
}

void CH32V003_SERVO_WriteCH4(uint16_t us)
{
    TIM_SetCompare4(TIM2, clamp_us(us));
}

void CH32V003_SERVO_InitTIM1CH4(void)
{
    if (s_tim1_initialized) return;
    s_tim1_initialized = 1;

    /* ----- Enable peripheral clocks ----- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_TIM1, ENABLE);

    /* ----- GPIO: PC4 (TIM1_CH4) as AF push-pull ----- */
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_4;
    gpio.GPIO_Mode  = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);

    /* ----- TIM1 time-base: 50 Hz, 1 us resolution ----- */
    TIM_TimeBaseInitTypeDef tb = {0};
    tb.TIM_Period        = SERVO_ARR;
    tb.TIM_Prescaler     = SERVO_PSC;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &tb);

    /* ----- CH4 (PC4): PWM mode 1, edge-aligned ----- */
    TIM_OCInitTypeDef oc = {0};
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = SERVO_DEFAULT;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM1, &oc);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* Generate update event to transfer preload → shadow registers */
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);

    /* TIM1 is an advanced-control timer: MOE must be set to enable outputs */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
}

void CH32V003_SERVO_WriteTIM1CH4(uint16_t us)
{
    TIM_SetCompare4(TIM1, clamp_us(us));
}
