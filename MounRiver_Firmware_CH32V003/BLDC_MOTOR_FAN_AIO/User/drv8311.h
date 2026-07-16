// MIT License
//
// Copyright (c) 2023 wirano
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//
// Created by wirano on 23-4-4.
//

#ifndef DRV8311_DRIVER_H
#define DRV8311_DRIVER_H

#include "drv8311_reg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    SPI = 0x00, tSPI = 0x01
} drv8311_protal_t;

typedef struct {
    struct {
        DRV8311_PWMCNTR_MODE_t mode;
        uint16_t period;
    } pwm_gen;

    struct {
        drv8311_protal_t protel;
        uint8_t devicd_id;
        uint8_t parity_check;

        void (*spi_trans)(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len);

        void (*nsleep_set)(uint8_t level);
    } interface;
} drv8311_instance_t;

typedef struct {
    DRV8311_PWMCNTR_MODE_t pwmcnt_mode;
    DRV8311_PWM_OSC_SYNC_t sync_mode;
    DRV8311_SPICLK_FREQ_SYNC_t spi_clk;
    DRV8311_SPISYNC_ACRCY_t spi_sync_clks;
    drv8311_protal_t portal;
    DRV8311_CSA_GAIN_t csa_gain;

    uint16_t pwm_period;
    uint8_t use_csa;
    uint8_t dev_id;
    uint8_t parity_check;

    void (*spi_trans)(uint8_t *send_data, uint8_t send_len, uint8_t *rec_data, uint8_t rec_len);

    void (*nsleep_set)(uint8_t level);
} drv8311_cfg_t;

typedef drv8311_instance_t *drv8311_handle_t;


// Returns 0 on success, or step number (1-4) where nFAULT went LOW:
//   1 = after nSLEEP wake, 2 = after SYS_CTRL, 3 = after PWMG_CTRL, 4 = after PWMG_PERIOD
int  drv8311_init(drv8311_handle_t *handle, drv8311_cfg_t *cfg);

void drv8311_nsleep_ctrl(drv8311_handle_t handle, uint8_t level);

drv8311_dev_sts1_t drv8311_get_status(drv8311_handle_t handle);

uint16_t drv8311_get_sync_period(drv8311_handle_t handle);

void drv8311_csa_ctrl(drv8311_handle_t handle, uint8_t en);

void drv8311_csa_set_gain(drv8311_handle_t handle, DRV8311_CSA_GAIN_t gain);

void drv8311_phase_ctrl(drv8311_handle_t handle, DRV8311_PHASE_MODE_t phase_a, DRV8311_PHASE_MODE_t phase_b,
                        DRV8311_PHASE_MODE_t phase_c);

void drv8311_out_ctrl(drv8311_handle_t handle, uint8_t en);

void drv8311_set_period(drv8311_handle_t handle, uint16_t period);

void drv8311_set_duty(drv8311_handle_t handle, float a, float b, float c);

void drv8311_set_duty_raw(drv8311_handle_t handle, uint16_t a, uint16_t b, uint16_t c);

void drv8311_set_duty_single(drv8311_handle_t handle, uint8_t phase_reg, uint16_t duty);

uint16_t drv8311_read_reg(drv8311_handle_t handle, uint8_t reg);

void drv8311_clear_faults(drv8311_handle_t handle);

void drv8311_write_reg(drv8311_handle_t handle, uint8_t reg, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif //DRV8311_DRIVER_H
