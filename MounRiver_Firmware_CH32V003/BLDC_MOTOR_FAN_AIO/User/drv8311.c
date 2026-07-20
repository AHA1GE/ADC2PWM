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

#include "debug.h"
#include "drv8311.h"
#include "drv8311_reg.h"

#define RW_CTRL_READ 0x01
#define RW_CTRL_WRITE 0x00

#define SWAP(x, y) do { (x) ^= (y); (y) ^= (x); (x) ^= (y); } while (0)


typedef struct {
    uint8_t parity: 1;
    uint8_t addr: 6;
    uint8_t rw_ctrl: 1;
} drv8311_spi_send_header_t;

typedef struct {
    uint16_t parity: 1;
    uint16_t zero_1: 2;
    uint16_t addr: 8;
    uint16_t device_id: 2;
    uint16_t zero_0: 2;
    uint16_t rw_ctrl: 1;
} drv8311_tspi_send_header_t;

typedef struct {
    uint16_t data: 15;
    uint16_t parity: 1;
} drv8311_send_data_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        drv8311_send_data_t data;
        drv8311_spi_send_header_t header;
    };
    uint8_t bytes[3];
} drv8311_spi_send_pkg_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        drv8311_send_data_t data;
        drv8311_tspi_send_header_t header;
    };
    uint8_t bytes[4];
} drv8311_tspi_send_pkg_t;

typedef union {
    struct __attribute__ ((__packed__)) {
        uint16_t data;
        uint8_t status;
    };
    uint8_t bytes[3];
} drv8311_recv_pkg_t;


static inline uint16_t parity_even_calc(uint16_t data) {
    uint16_t parity = 0;

    while (data) {
        parity ^= data & 1;
        data >>= 1;
    }

    return parity & 1;
}

static inline drv8311_spi_send_header_t drv8311_spi_header_gen(uint8_t rw_mode, uint8_t addr) {
    drv8311_spi_send_header_t header;
    uint16_t *temp;

    header.rw_ctrl = rw_mode;
    header.addr = addr;

    temp = (uint16_t *) &header;

    header.parity = parity_even_calc(*temp);

    return header;
}

static inline drv8311_tspi_send_header_t drv8311_tspi_header_gen(uint8_t rw_mode, uint8_t addr, uint8_t device_id) {
    drv8311_tspi_send_header_t header;
    uint16_t *temp;

    header.rw_ctrl = rw_mode;
    header.device_id = device_id;
    header.addr = addr;

    temp = (uint16_t *) &header;

    header.parity = parity_even_calc(*temp);

    return header;
}

// ===================================================================
//  DRV8311P tSPI layer — correct 32-bit frame format per datasheet.
//
//  Bit 31:    R/W   (0 = Write, 1 = Read)
//  Bit 30-27: Device ID (4 bits, typically 0)
//  Bit 26-19: Register address (8 bits)
//  Bit 18-17: Reserved (00)
//  Bit 16:    Header parity (even, over bits 31-17)
//  Bit 15-1:  Data (15 bits of 16-bit register value)
//  Bit 0:     Data parity (even, over bits 15-1)
//
//  Write: R/W=0, data = register value.  Response = 3 bytes (status+data).
//  Read:  R/W=1, data = 0x0000.  Response = 3 bytes (status+data).
// ===================================================================

// Build and send a 4-byte tSPI frame, receive 3-byte response.
// tx_frame[4] will be modified (byte-swapped to big-endian for MSB-first SPI).
static void drv8311_tspi_xfer(drv8311_handle_t handle,
                              uint8_t rw, uint8_t reg, uint16_t data,
                              uint8_t *rx3) {
    uint8_t dev_id = handle->interface.devicd_id & 0x0F;
    uint8_t tx[4];

    // Build 32-bit frame (little-endian in tx[0..3])
    uint32_t frame = 0;
    frame |= ((uint32_t)rw            << 31);   // bit 31
    frame |= ((uint32_t)dev_id        << 27);   // bits 30-27
    frame |= ((uint32_t)(reg & 0xFF)  << 19);   // bits 26-19
    // bits 18-17 = reserved = 00
    frame |= ((uint32_t)(data & 0xFFFF));        // bits 15-0

    // Compute header parity (bits 31-17)
    uint8_t hp = 0;
    for (int b = 17; b < 32; b++)
        if ((frame >> b) & 1) hp ^= 1;
    frame |= ((uint32_t)hp << 16);               // bit 16

    // Compute data parity (bits 15-1)
    uint8_t dp = 0;
    for (int b = 1; b < 16; b++)
        if ((frame >> b) & 1) dp ^= 1;
    frame |= ((uint32_t)dp);                     // bit 0

    // Pack big-endian into tx[] for MSB-first SPI
    tx[0] = (frame >> 24) & 0xFF;
    tx[1] = (frame >> 16) & 0xFF;
    tx[2] = (frame >> 8)  & 0xFF;
    tx[3] =  frame        & 0xFF;

    uint8_t rx[4] = {0};
    handle->interface.spi_trans(tx, 4, rx, 4);

    // Response is shifted by 1 byte: the DRV8311 decodes the header
    // during byte 0 and starts outputting SDO at byte 1.
    // rx[0]=garbage, rx[1]=status, rx[2]=data_hi, rx[3]=data_lo
    if (rx3) {
        rx3[0] = rx[1];  // status
        rx3[1] = rx[2];  // data high
        rx3[2] = rx[3];  // data low
    }
}

static void drv8311_write(drv8311_handle_t handle, uint8_t reg, uint16_t data) {
    uint8_t rx[3];
    drv8311_tspi_xfer(handle, 0, reg, data, rx);
}

static uint16_t drv8311_read(drv8311_handle_t handle, uint8_t reg) {
    uint8_t rx[3];
    drv8311_tspi_xfer(handle, 1, reg, 0x0000, rx);
    // rx[0]=status, rx[1]=data_hi, rx[2]=data_lo
    return ((uint16_t)rx[1] << 8) | rx[2];
}

// Returns 0 on success, or the step number (1-5) where nFAULT went LOW.
int drv8311_init(drv8311_handle_t *handle, drv8311_cfg_t *cfg) {
    drv8311_reg_t reg;

    // Static allocation — avoids malloc(NULL) → HardFault on CH32V003's tiny heap.
    static drv8311_instance_t inst_buf;
    drv8311_instance_t *dev = &inst_buf;

    dev->interface.protel = cfg->portal;
    dev->interface.devicd_id = cfg->dev_id;
    dev->interface.parity_check = cfg->parity_check;
    dev->interface.spi_trans = cfg->spi_trans;
    dev->interface.nsleep_set = cfg->nsleep_set;

    // Make handle valid now so caller can use it for diagnostics
    *handle = dev;

    // --- Step 1: nSLEEP reset pulse ---
    // nSLEEP is hardwired to BAT+ on this board — skip toggle.
    // Just wait for DRV8311 power-on stabilization.
    Delay_Ms(500);                  // extended stabilise
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) return 1;

    // --- Step 2: Unlock DRV8311 registers ---
    reg.half_word = 0;
    reg.sys_ctrl.write_key = 0x03;  // 011b = unlock key
    reg.sys_ctrl.reg_lock   = 0;
    reg.sys_ctrl.spi_pen    = 0;
    drv8311_write(dev, DRV8311_SYS_CTRL_ADDR, reg.half_word);

    // --- Step 2b: Clear any latched faults before they trigger nFAULT ---
    drv8311_write(dev, DRV8311_FLT_CLR_ADDR, 0x0001);
    Delay_Ms(5);

    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) return 2;

    // --- Step 3: PWM generator config ---
    reg.half_word = 0;
    dev->pwm_gen.mode = cfg->pwmcnt_mode;
    reg.pwmg_ctrl.pwmcntr_mode = cfg->pwmcnt_mode;
    reg.pwmg_ctrl.pwm_osc_sync = cfg->sync_mode;
    if (cfg->sync_mode == USE_SPI_CLK) {
        reg.pwmg_ctrl.spiclk_freq_sync = cfg->spi_clk;
        reg.pwmg_ctrl.spisync_acrcy = cfg->spi_sync_clks;
    }
    drv8311_write(dev, DRV8311_PWMG_CTRL_ADDR, reg.half_word);
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) return 3;

    // --- Step 4: PWM period ---
    reg.half_word = 0;
    dev->pwm_gen.period = cfg->pwm_period;
    reg.pwmg_period.pwm_prd_out = cfg->pwm_period;
    drv8311_write(dev, DRV8311_PWMG_PERIOD_ADDR, reg.half_word);
    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) return 4;

    // --- Step 5: PWM mode select (PWM_CTRL1) ---
    // PWM_MODE = 11b (PWM Generator Mode), SSC disabled.
    // DRV8311 reset default is Hi-Z (00b) — MUST be written.
    // The previous tSPI driver had broken framing that caused SPI_FLT
    // on this write; the corrected 32-bit tSPI protocol now works.
    reg.half_word = 0;
    reg.pwm_ctrl1.pwm_mode = 3;       // PWM Generator Mode (0b11)
    reg.pwm_ctrl1.ssc_dis  = 1;       // Disable spread-spectrum clocking
    drv8311_write(dev, DRV8311_PWM_CTRL1_ADDR, reg.half_word);

    // Readback: verify PWM_CTRL1 was accepted
    uint16_t ctrl1_rb = drv8311_read(dev, DRV8311_PWM_CTRL1_ADDR);
    if ((ctrl1_rb & 0x0007) != 0x0007) return 5;

    if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) {
        // nFAULT triggered — check status, clear, retry once
        uint16_t sts1 = drv8311_read(dev, DRV8311_DEV_STS1_ADDR);
        drv8311_write(dev, DRV8311_FLT_CLR_ADDR, 0x0001);
        Delay_Ms(2);
        // Retry
        drv8311_write(dev, DRV8311_PWM_CTRL1_ADDR, reg.half_word);
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7) == Bit_RESET) {
            (void)sts1;
            return 5;
        }
    }

    // --- CSA config (optional) ---
    reg.half_word = 0;
    if (cfg->use_csa) {
        reg.csa_ctrl.csa_en = 1;
        reg.csa_ctrl.csa_gain = cfg->csa_gain;
        drv8311_write(dev, DRV8311_CSA_CTRL_ADDR, reg.half_word);
    }

    reg.half_word = 0;
    if (cfg->parity_check) {
        //todo
    }

    return 0;  // success
}

void drv8311_nsleep_ctrl(drv8311_handle_t handle, uint8_t level) {
    handle->interface.nsleep_set(level);
}

drv8311_dev_sts1_t drv8311_get_status(drv8311_handle_t handle) {
    drv8311_reg_t rec;
    rec.half_word = drv8311_read(handle, DRV8311_DEV_STS1_ADDR);

    return rec.dev_sts1;
}

uint16_t drv8311_get_sync_period(drv8311_handle_t handle) {
    drv8311_reg_t rec;
    rec.half_word = drv8311_read(handle, DRV8311_PWM_SYNC_PRD_ADDR);

    return rec.pwm_sync_prd.pwm_sync_prd;
}

void drv8311_csa_ctrl(drv8311_handle_t handle, uint8_t en) {
    drv8311_reg_t csa_ctrl;

    csa_ctrl.half_word = drv8311_read(handle, DRV8311_CSA_CTRL_ADDR);

    if (en) {
        csa_ctrl.csa_ctrl.csa_en = 1;
    } else {
        csa_ctrl.csa_ctrl.csa_en = 0;
    }

    drv8311_write(handle, DRV8311_CSA_CTRL_ADDR, csa_ctrl.half_word);
}

void drv8311_csa_set_gain(drv8311_handle_t handle, DRV8311_CSA_GAIN_t gain) {
    drv8311_reg_t csa_ctrl;

    csa_ctrl.half_word = drv8311_read(handle, DRV8311_CSA_CTRL_ADDR);

    csa_ctrl.csa_ctrl.csa_gain = gain;

    drv8311_write(handle, DRV8311_CSA_CTRL_ADDR, csa_ctrl.half_word);
}

void drv8311_phase_ctrl(drv8311_handle_t handle, DRV8311_PHASE_MODE_t phase_a, DRV8311_PHASE_MODE_t phase_b,
                        DRV8311_PHASE_MODE_t phase_c) {
    drv8311_reg_t phase_ctrl;

    phase_ctrl.pwm_state.pwma_state = phase_a;
    phase_ctrl.pwm_state.pwmb_state = phase_b;
    phase_ctrl.pwm_state.pwmc_state = phase_c;

    drv8311_write(handle, DRV8311_PWM_STATE_ADDR, phase_ctrl.half_word);
}

void drv8311_out_ctrl(drv8311_handle_t handle, uint8_t en) {
    drv8311_reg_t pwmg_ctrl;

    pwmg_ctrl.half_word = drv8311_read(handle, DRV8311_PWMG_CTRL_ADDR);

    if (en) {
        pwmg_ctrl.pwmg_ctrl.pwm_en = 1;
    } else {
        pwmg_ctrl.pwmg_ctrl.pwm_en = 0;
    }

    drv8311_write(handle, DRV8311_PWMG_CTRL_ADDR, pwmg_ctrl.half_word);
}

void drv8311_set_period(drv8311_handle_t handle, uint16_t period) {
    drv8311_reg_t period_reg;

    period_reg.pwmg_period.pwm_prd_out = period;
    drv8311_write(handle, DRV8311_PWMG_PERIOD_ADDR, period_reg.half_word);
}

void drv8311_set_duty(drv8311_handle_t handle, float a, float b, float c) {
    drv8311_reg_t pwmg_duty;
    uint16_t cmp_a, cmp_b, cmp_c;

    cmp_a = (uint16_t) ((float) handle->pwm_gen.period * a);
    cmp_b = (uint16_t) ((float) handle->pwm_gen.period * b);
    cmp_c = (uint16_t) ((float) handle->pwm_gen.period * c);

    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_a;
    drv8311_write(handle, DRV8311_PWMG_A_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_b;
    drv8311_write(handle, DRV8311_PWMG_B_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = cmp_c;
    drv8311_write(handle, DRV8311_PWMG_C_DUTY_ADDR, pwmg_duty.half_word);
}

void drv8311_set_duty_raw(drv8311_handle_t handle, uint16_t a, uint16_t b, uint16_t c) {
    drv8311_reg_t pwmg_duty = {0};

    // Clamp to 12-bit register width (0-4095)
    if (a > 0x0FFF) a = 0x0FFF;
    if (b > 0x0FFF) b = 0x0FFF;
    if (c > 0x0FFF) c = 0x0FFF;

    pwmg_duty.pwmg_x_duty.pwm_duty_outx = a;
    drv8311_write(handle, DRV8311_PWMG_A_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = b;
    drv8311_write(handle, DRV8311_PWMG_B_DUTY_ADDR, pwmg_duty.half_word);
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = c;
    drv8311_write(handle, DRV8311_PWMG_C_DUTY_ADDR, pwmg_duty.half_word);
}

void drv8311_set_duty_single(drv8311_handle_t handle, uint8_t phase_reg, uint16_t duty) {
    drv8311_reg_t pwmg_duty = {0};
    if (duty > 0x0FFF) duty = 0x0FFF;
    pwmg_duty.pwmg_x_duty.pwm_duty_outx = duty;
    drv8311_write(handle, phase_reg, pwmg_duty.half_word);
}

uint16_t drv8311_read_reg(drv8311_handle_t handle, uint8_t reg) {
    return drv8311_read(handle, reg);
}

void drv8311_clear_faults(drv8311_handle_t handle) {
    // Write 1 to FLT_CLR bit (bit 0) — auto-clears after the write.
    drv8311_write(handle, DRV8311_FLT_CLR_ADDR, 0x0001);
}

void drv8311_write_reg(drv8311_handle_t handle, uint8_t reg, uint16_t data) {
    drv8311_write(handle, reg, data);
}
