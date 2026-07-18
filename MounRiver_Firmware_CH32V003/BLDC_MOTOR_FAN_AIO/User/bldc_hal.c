/*===========================================================================
 * bldc_hal.c — Hardware Abstraction Layer for BLDC_MOTOR_FAN_AIO
 *
 * Pin assignments per PIN.md. CH32V003F4P6 TSSOP-20, 48MHz HSE.
 *===========================================================================*/

#include "bldc_hal.h"
#include "drv8311_reg.h"
#include "debug.h"

/*===========================================================================
 * Global State
 *===========================================================================*/

drv8311_handle_t drv = NULL;
volatile uint32_t sys_tick_millis = 0;
volatile uint32_t tim1_tick_count = 0;

static uint16_t batt_adc_ema;   // EMA of battery ADC (×1, no fixed-point needed)
static uint8_t  batt_initialized;

/*===========================================================================
 * GPIO Initialization — All 20 pins per PIN.md
 *===========================================================================*/

void HAL_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* --- PORTA: Analog inputs --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;  // PA1, PA2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* --- PORTC: Mixed analog + digital + AF --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // PC0 (Btn_Down) — Input with pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // PC1 (OLED_SDA), PC2 (OLED_SCL) — configured in OLED_Init()
    // Leave as default; OLED_Init() will reconfigure.

    // PC3 (SPI_CS) — Push-pull output, high (inactive)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_SetBits(GPIOC, GPIO_Pin_3);

    // PC4 (ESC_CURR_U) — Analog input
    // PC5 (SPI_SCK), PC6 (SPI_MOSI) — configured in HAL_SPI_Init()
    // PC7 (SPI_MISO) — configured in HAL_SPI_Init()
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* --- PORTD: Mixed digital + analog --- */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // PD0 (Btn_Up) — Input with pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // PD1 (LED/SWD) — push-pull output, configured LAST in main()
    //   after Delay_Ms(100) SWD window. Not set here.

    // PD2, PD3, PD4, PD5, PD6 — Analog inputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 |
                                  GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // PD7 (nFAULT) — Floating input (DRV8311 open-drain output, pulled up
    //   externally by R1=10kΩ on BAT+/3V3 net)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*===========================================================================
 * LED — PD1 (shared SWDIO), active low
 *===========================================================================*/

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_SetBits(GPIOD, GPIO_Pin_1);  // Off (active low)
}

void LED_On(void)  { GPIO_ResetBits(GPIOD, GPIO_Pin_1); }
void LED_Off(void) { GPIO_SetBits(GPIOD, GPIO_Pin_1); }
void LED_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_1))
        GPIO_ResetBits(GPIOD, GPIO_Pin_1);
    else
        GPIO_SetBits(GPIOD, GPIO_Pin_1);
}

/*===========================================================================
 * Hardware SPI1 — PC5(SCK), PC6(MOSI), PC7(MISO), PC3(CS=GPIO)
 *
 * Reference: official_demo_CH32V003/SPI/2Lines_FullDuplex
 *===========================================================================*/

void HAL_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE);

    // SCK (PC5) — Alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // MOSI (PC6) — Alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // MISO (PC7) — Floating input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // SPI1 config: Master, 8-bit, MSB first, CPOL=0 CPHA=1, NSS soft
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  // 48/8=6MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

// tSPI transfer callback for drv8311.c
// Sends tx_len bytes, receives rx_len bytes, MSB-first.
// CS is toggled manually because tSPI is per-frame (not per-byte) CS control.
void spi_trans_cb(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t rx_len)
{
    uint8_t i;

    GPIO_ResetBits(GPIOC, PIN_SPI_CS);  // CS active low

    // Send all TX bytes, read RX in parallel
    for (i = 0; i < tx_len; i++) {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI1, tx[i]);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        uint8_t rx_byte = SPI_I2S_ReceiveData(SPI1);
        if (rx && i < rx_len) rx[i] = rx_byte;
    }

    // If we need more RX bytes than TX (only for SPI portal, not tSPI),
    // clock out dummy bytes. For tSPI, tx_len == rx_len == 4.
    for (i = tx_len; i < rx_len; i++) {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI1, 0x00);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        if (rx) rx[i] = SPI_I2S_ReceiveData(SPI1);
    }

    GPIO_SetBits(GPIOC, PIN_SPI_CS);    // CS inactive high
}

// nSLEEP — hardwired to BAT+ via 20kΩ (R20) on this board. No-op.
void nsleep_set_cb(uint8_t level)
{
    (void)level;
}

/*===========================================================================
 * DRV8311 Initialization
 *===========================================================================*/

int DRV8311_Init(void)
{
    drv8311_cfg_t cfg = {0};

    cfg.pwmcnt_mode  = UP;
    cfg.sync_mode    = SYNC_DISABLE;
    cfg.portal       = tSPI;
    cfg.csa_gain     = CSA_GAIN_500MV;
    cfg.pwm_period   = DRV_PWM_PERIOD;
    cfg.use_csa      = 1;
    cfg.dev_id       = 0;
    cfg.parity_check = 0;
    cfg.spi_trans    = spi_trans_cb;
    cfg.nsleep_set   = nsleep_set_cb;

    return drv8311_init(&drv, &cfg);
}

void DRV8311_Enable(void)
{
    if (!drv) return;
    drv8311_out_ctrl(drv, 1);
}

void DRV8311_Disable(void)
{
    if (!drv) return;
    drv8311_out_ctrl(drv, 0);
}

/*===========================================================================
 * ADC1 — All 8 analog channels, single-channel mode, 6MHz ADCCLK
 *===========================================================================*/

void HAL_ADC_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);  // 48MHz / 8 = 6MHz ADCCLK

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;        // Single channel per conversion
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // One conversion per trigger
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // Software trigger
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Start with battery channel (safe default)
    ADC_RegularChannelConfig(ADC1, ADC_CH_BATT, 1, ADC_SampleTime_73Cycles);

    ADC_Cmd(ADC1, ENABLE);

    // Calibration
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

/*===========================================================================
 * TIM1 — Commutation timing reference at ~16kHz (period ~63µs)
 *   PCLK2 = 48MHz, PSC=47 → 1MHz, ARR=62 → 1MHz/63 = 15.87kHz
 *===========================================================================*/

void HAL_TIM1_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseStructure.TIM_Prescaler = 48 - 1;   // 48MHz / 48 = 1MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 63 - 1;       // 1MHz / 63 = 15.87kHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // TRGO on update (for potential ADC triggering — not used yet, but configured)
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    // Enable update interrupt for commutation timing
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    // Don't start TIM1 yet — started when entering RUN state
}

void HAL_TIM1_Start(void) { TIM_Cmd(TIM1, ENABLE); }
void HAL_TIM1_Stop(void)  { TIM_Cmd(TIM1, DISABLE); }

/*===========================================================================
 * Battery ADC — Periodic sampling, EMA filter
 *===========================================================================*/

void BAT_ADC_Sample(void)
{
    // Temporarily switch to battery channel, sample, restore.
    // Save current channel config to restore after (simplified: just set back to
    // whatever the motor control layer expects — bldc_control manages BEMF channel).
    ADC_RegularChannelConfig(ADC1, ADC_CH_BATT, 1, ADC_SampleTime_73Cycles);

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    uint16_t val = ADC_GetConversionValue(ADC1);
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    // EMA filter: α = 0.25
    if (!batt_initialized) {
        batt_adc_ema = val * 256;  // Fixed-point ×256
        batt_initialized = 1;
    } else {
        // ema = (3 * ema + 1 * val) / 4  → α = 0.25
        batt_adc_ema = ((batt_adc_ema * 3) >> 2) + (val * 64);
    }
}

uint16_t BAT_Get_mV(void)
{
    if (!batt_initialized) return 0;
    // batt_adc_ema is ×256 fixed-point. Divide by 256 to get raw ADC.
    // V_BAT_mV = ADC_raw * 3300 / 1024 / (4.7 / 14.7)
    //          = ADC_raw * 3300 * 147 / (1024 * 47)
    //          = ADC_raw * 485100 / 48128
    //          ≈ ADC_raw * 10.08
    uint16_t adc_raw = (uint16_t)(batt_adc_ema >> 8);
    return (uint16_t)((uint32_t)adc_raw * 1008 / 100);
}

/*===========================================================================
 * Button Handling — Debounced, with edge detection
 *
 * Call BTN_Process() every BTN_POLL_MILLIS (10ms).
 * State is internal; BTN_Up_Pressed() / BTN_Down_Pressed() return true on the
 * first debounced falling edge ONLY — they auto-clear after being read.
 *===========================================================================*/

typedef struct {
    uint8_t  stable_count;      // Consecutive same-readings counter
    uint8_t  debounced_state;   // 1 = not pressed (pull-up), 0 = pressed
    uint8_t  last_raw;          // Previous raw reading
    uint8_t  edge_detected;     // 1 = falling edge detected this cycle
    uint32_t press_start_millis;// When press began
    uint8_t  long_press_active; // 1 = long-press mode
    uint32_t last_repeat_millis;// Last auto-repeat action
} button_ctx_t;

static button_ctx_t btn_up_ctx;
static button_ctx_t btn_down_ctx;

#define BTN_DEBOUNCE_COUNT      3    // 3 × 10ms = 30ms debounce
#define BTN_LONG_PRESS_MILLIS   500
#define BTN_REPEAT_MILLIS       100

static void btn_update(button_ctx_t *ctx, uint8_t raw_state)
{
    // raw_state: 0 = pressed (active low with pull-up), 1 = released
    if (raw_state == ctx->last_raw) {
        if (ctx->stable_count < BTN_DEBOUNCE_COUNT) {
            ctx->stable_count++;
            if (ctx->stable_count >= BTN_DEBOUNCE_COUNT) {
                // State is now stable
                if (ctx->debounced_state == 1 && raw_state == 0) {
                    // Falling edge: just pressed
                    ctx->debounced_state = 0;
                    ctx->edge_detected = 1;
                    ctx->press_start_millis = sys_tick_millis;
                    ctx->long_press_active = 0;
                    ctx->last_repeat_millis = 0;
                } else if (ctx->debounced_state == 0 && raw_state == 1) {
                    // Rising edge: released
                    ctx->debounced_state = 1;
                    ctx->long_press_active = 0;
                }
            }
        } else {
            // Already stable — check long press
            if (ctx->debounced_state == 0) {  // Pressed
                uint32_t held = sys_tick_millis - ctx->press_start_millis;
                if (held >= BTN_LONG_PRESS_MILLIS) {
                    if (!ctx->long_press_active) {
                        ctx->long_press_active = 1;
                        ctx->last_repeat_millis = sys_tick_millis;
                        ctx->edge_detected = 1;  // First long-press tick
                    } else {
                        uint32_t since_repeat = sys_tick_millis - ctx->last_repeat_millis;
                        if (since_repeat >= BTN_REPEAT_MILLIS) {
                            ctx->last_repeat_millis = sys_tick_millis;
                            ctx->edge_detected = 1;  // Repeat tick
                        }
                    }
                }
            }
        }
    } else {
        ctx->stable_count = 0;
        ctx->last_raw = raw_state;
    }
}

void BTN_Process(void)
{
    uint8_t up_raw   = (GPIO_ReadInputDataBit(GPIOD, PIN_BTN_UP) == Bit_RESET) ? 0 : 1;
    uint8_t down_raw = (GPIO_ReadInputDataBit(GPIOC, PIN_BTN_DOWN) == Bit_RESET) ? 0 : 1;

    btn_update(&btn_up_ctx, up_raw);
    btn_update(&btn_down_ctx, down_raw);
}

bool BTN_Up_Pressed(void)
{
    if (btn_up_ctx.edge_detected) {
        btn_up_ctx.edge_detected = 0;
        return true;
    }
    return false;
}

bool BTN_Down_Pressed(void)
{
    if (btn_down_ctx.edge_detected) {
        btn_down_ctx.edge_detected = 0;
        return true;
    }
    return false;
}

bool BTN_Up_IsLongPress(void)
{
    return btn_up_ctx.long_press_active;
}

bool BTN_Down_IsLongPress(void)
{
    return btn_down_ctx.long_press_active;
}
