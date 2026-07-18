/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v00x_it.h
 * Author             : BLDC_MOTOR_FAN_AIO
 * Version            : V1.0.0
 * Date               : 2026-07-18
 * Description        : Interrupt handler declarations.
 *******************************************************************************/

#ifndef __CH32V00x_IT_H
#define __CH32V00x_IT_H

#include <ch32v00x.h>
#include "debug.h"

/* Global timing counters */
extern volatile uint32_t sys_tick_millis;
extern volatile uint32_t tim1_tick_count;

#endif /* __CH32V00x_IT_H */
