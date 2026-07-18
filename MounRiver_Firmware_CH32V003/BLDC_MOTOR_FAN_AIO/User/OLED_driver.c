/*========================================================================*/
/*========================================================================*/
// SSD1306 driver optimized for:
// Controller RAM : 128x64
// Visible pixels : 88x48
// Hidden area    : first 40 columns + first 8 rows
//
// Note:
// The software layer (OLED.c/OLED.h) uses a logical 128x56 framebuffer.
// To keep indexing/layout consistent across translation units, the exported
// framebuffer here must keep the same logical dimensions.
/*========================================================================*/

#include "OLED_driver.h"

/*================================[CONFIG]================================*/


#define OLED_VISIBLE_WIDTH        (88)
#define OLED_VISIBLE_HEIGHT       (48)

#define OLED_LOGICAL_WIDTH        (128)
#define OLED_LOGICAL_HEIGHT       (48)

#define OLED_X_OFFSET             (40)
#define OLED_Y_OFFSET             (8)

#define OLED_PAGE_NUM             (OLED_VISIBLE_HEIGHT / 8)
#define OLED_LOGICAL_PAGE_NUM     (OLED_LOGICAL_HEIGHT / 8)

#define OLED_CMD                  (0)
#define OLED_DATA                 (1)

#define OLED_SDA_Clr()  GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define OLED_SDA_Set()  GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define OLED_SCL_Clr()  GPIO_ResetBits(GPIOC, GPIO_Pin_2)
#define OLED_SCL_Set()  GPIO_SetBits(GPIOC, GPIO_Pin_2)

/*========================================================================*/

/* Keep this shape aligned with extern declaration in OLED.c */
uint8_t OLED_DisplayBuf[OLED_LOGICAL_PAGE_NUM][OLED_LOGICAL_WIDTH];

bool OLED_ColorMode = true;

/**
 * @brief 设置显示模式
 */
void OLED_SetColorMode(bool colormode)
{
    OLED_ColorMode = colormode;
}

/**
 * @brief I2C start
 */
static void OLED_I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();

    OLED_SDA_Clr();
    OLED_SCL_Clr();
}

/**
 * @brief I2C stop
 */
static void OLED_I2C_Stop(void)
{
    OLED_SDA_Clr();

    OLED_SCL_Set();
    OLED_SDA_Set();
}

/**
 * @brief write byte
 */
static void OLED_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            OLED_SDA_Set();
        }
        else
        {
            OLED_SDA_Clr();
        }

        OLED_SCL_Set();
        OLED_SCL_Clr();

        data <<= 1;
    }

    /* ignore ACK */
    OLED_SCL_Set();
    OLED_SCL_Clr();
}

/**
 * @brief write command
 */
static void OLED_Write_CMD(uint8_t cmd)
{
    OLED_I2C_Start();

    OLED_WriteByte(0x78);
    OLED_WriteByte(0x00);
    OLED_WriteByte(cmd);

    OLED_I2C_Stop();
}

/**
 * @brief write data buffer
 */
static void OLED_WriteDataArr(uint8_t *Data, uint8_t Count)
{
    uint8_t i;

    OLED_I2C_Start();

    OLED_WriteByte(0x78);
    OLED_WriteByte(0x40);

    if (OLED_ColorMode)
    {
        for (i = 0; i < Count; i++)
        {
            OLED_WriteByte(Data[i]);
        }
    }
    else
    {
        for (i = 0; i < Count; i++)
        {
            OLED_WriteByte(~Data[i]);
        }
    }

    OLED_I2C_Stop();
}

/**
 * @brief set cursor
 */
static void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    X += OLED_X_OFFSET;
    Page += (OLED_Y_OFFSET / 8);

    OLED_Write_CMD(0xB0 | Page);
    OLED_Write_CMD(0x10 | ((X & 0xF0) >> 4));
    OLED_Write_CMD(0x00 | (X & 0x0F));
}

/**
 * @brief full refresh
 */
void OLED_Update(void)
{
    uint8_t page;

    for (page = 0; page < OLED_PAGE_NUM; page++)
    {
        OLED_SetCursor(page, 0);

        OLED_WriteDataArr(
            OLED_DisplayBuf[page],
            OLED_VISIBLE_WIDTH
        );
    }
}

/**
 * @brief partial refresh
 */
void OLED_UpdateArea(
    int16_t X,
    int16_t Y,
    int16_t Width,
    int16_t Height
)
{
    int16_t j;

    if (X > OLED_VISIBLE_WIDTH - 1)
    {
        return;
    }

    if (Y > OLED_VISIBLE_HEIGHT - 1)
    {
        return;
    }

    if (X < 0)
    {
        X = 0;
    }

    if (Y < 0)
    {
        Y = 0;
    }

    if ((X + Width) > OLED_VISIBLE_WIDTH)
    {
        Width = OLED_VISIBLE_WIDTH - X;
    }

    if ((Y + Height) > OLED_VISIBLE_HEIGHT)
    {
        Height = OLED_VISIBLE_HEIGHT - Y;
    }

    for (j = (Y / 8);
         j < (((Y + Height - 1) / 8) + 1);
         j++)
    {
        OLED_SetCursor(j, X);

        OLED_WriteDataArr(
            &OLED_DisplayBuf[j][X],
            Width
        );
    }
}

/* external */
extern void OLED_Clear(void);

/**
 * @brief init
 */
void OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_1 | GPIO_Pin_2;

    GPIO_InitStructure.GPIO_Mode =
        GPIO_Mode_Out_OD;

    GPIO_InitStructure.GPIO_Speed =
        GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    OLED_SCL_Set();
    OLED_SDA_Set();

    OLED_Write_CMD(0xAE);

    OLED_Write_CMD(0xD5);
    OLED_Write_CMD(0x80);

    OLED_Write_CMD(0xA8);
    OLED_Write_CMD(0x3F);

    OLED_Write_CMD(0xD3);
    OLED_Write_CMD(0x00);

    OLED_Write_CMD(0x40);

    OLED_Write_CMD(0xA1);

    OLED_Write_CMD(0xC8);

    OLED_Write_CMD(0xDA);
    OLED_Write_CMD(0x12);

    OLED_Write_CMD(0x81);
    OLED_Write_CMD(0xCF);

    OLED_Write_CMD(0xD9);
    OLED_Write_CMD(0xF1);

    OLED_Write_CMD(0xDB);
    OLED_Write_CMD(0x30);

    OLED_Write_CMD(0xA4);

    OLED_Write_CMD(0xA6);

    OLED_Write_CMD(0x8D);
    OLED_Write_CMD(0x14);

    OLED_Clear();

    OLED_Update();

    OLED_Write_CMD(0xAF);
}

/**
 * @brief brightness
 */
void OLED_SetBrightness(int16_t Brightness)
{
    if (Brightness > 255)
    {
        Brightness = 255;
    }

    if (Brightness < 0)
    {
        Brightness = 0;
    }

    OLED_Write_CMD(0x81);
    OLED_Write_CMD((uint8_t)Brightness);
}