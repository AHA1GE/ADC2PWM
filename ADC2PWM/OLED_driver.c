/*========================================================================*/
/*========================================================================*/
// 这个头文件是OLED-Basic-Lib库的 [硬件层] 实现文件，移植的时候需要更改这个文件的内容
// 版本v1.0.0
/*========================================================================*/
/*========================================================================*/

#include "OLED_driver.h"

/*========================================================================*/
/*================================[可配置宏]===============================*/
/*========================================================================*/
// 这些宏无需作为接口配置，所以不需要在头文件中声明，直接在源文件中定义即可。
// 是否开启动态刷新。如果开启，OLED-Basic-Lib会仅在显存更新的时候刷新有变化的区域，提高效率。
// 在多数情况下，建议开启动态刷新，以提高显示效率。
#define IF_ENABLE_DYNAMIC_REFRESH       (false)
#define DYNAMIC_REFRESH_LENGHT          (9)   // 动态刷新区块的长度，单位为像素。

#define OLED_HEIGHT_DRIVER	        	(40)					//OLED像素的高度
#define OLED_WIDTH_DRIVER		    	(72)					//OLED像素的宽度
#define OLED_PAGE_DRIVER				(OLED_HEIGHT_DRIVER/8)	//OLED的页数（由高度自动计算）

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

// port to ch32v003
#define OLED_SCL_Clr()  GPIO_ResetBits(GPIOC, GPIO_Pin_1);for(int i=0;i<6;i++)   // 复位 SCL (将 GPIOC 的 1 号引脚拉低)
#define OLED_SCL_Set()  GPIO_SetBits(GPIOC, GPIO_Pin_1);for(int i=0;i<6;i++)  // 置位 SCL (将 GPIOC 的 1 号引脚拉高)
#define OLED_SDA_Clr()  GPIO_ResetBits(GPIOC, GPIO_Pin_2);for(int i=0;i<3;i++)   // 复位 SDA (将 GPIOC 的 2 号引脚拉低)
#define OLED_SDA_Set()  GPIO_SetBits(GPIOC, GPIO_Pin_2);for(int i=0;i<3;i++)  // 置位 SDA (将 GPIOC 的 2 号引脚拉高)

/*========================================================================*/
/*========================================================================*/
/*========================================================================*/

// 显存数组，OLED-Basic-Lib的绘制函数都是对这个数组进行操作
uint8_t OLED_DisplayBuf[OLED_HEIGHT_DRIVER/8][OLED_WIDTH_DRIVER];

bool OLED_ColorMode = true;			//OLED的颜色模式
bool OLED_IfUpdate=false;			//是否已经更新显存

// 如果用户定义了动态刷新，则创建动态刷新区块校验数组
#if IF_ENABLE_DYNAMIC_REFRESH
// 此次校验值
uint16_t page_checksum[OLED_PAGE_DRIVER][OLED_WIDTH_DRIVER/DYNAMIC_REFRESH_LENGHT] = {0}; 
// 上一次校验值
uint16_t previous_page_checksum[OLED_PAGE_DRIVER][OLED_WIDTH_DRIVER/DYNAMIC_REFRESH_LENGHT] = {0}; 
#endif


/**
 * @brief 设置显示模式
 * @param colormode true: 黑色模式，false: 白色模式
 * @return 无
 */
void OLED_SetColorMode(bool colormode){
	OLED_ColorMode = colormode;
}

/**
 * @brief 此次刷新是否有更新显存，用于计算帧率
 * @return true: 有更新，false: 无更新
 */
bool OLED_IfChangedScreen(void){
	return OLED_IfUpdate;
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void OLED_DelayMs(uint32_t xms)
{
	// while(xms--)
	// {
	// 	SysTick->LOAD = 72 * 1000;				//设置定时器重装值
	// 	SysTick->VAL = 0x00;					//清空当前计数值
	// 	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	// 	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	// 	SysTick->CTRL = 0x00000004;				//关闭定时器
	// }
    delay(xms);
}
/**
 * @brief IIC开始信号
 * @return 无
 */
void OLED_I2C_Start(void)
{
	
	OLED_SDA_Set();		//释放SDA，确保SDA为高电平
	OLED_SCL_Set();		//释放SCL，确保SCL为高电平
	OLED_SDA_Clr();		//在SCL高电平期间，拉低SDA，产生起始信号
	OLED_SCL_Clr();		//起始后把SCL也拉低，即为了占用总线，也为了方便总线时序的拼接
}

/**
 * @brief IIC停止信号
 * @return 无
 */
void OLED_I2C_Stop(void)
{
	
	OLED_SDA_Clr();		//拉低SDA，确保SDA为低电平
	OLED_SCL_Set();		//释放SCL，使SCL呈现高电平
	OLED_SDA_Set();		//在SCL高电平期间，释放SDA，产生终止信号
}
/**
 * @brief OLED写1字节数据
 * @param data 要写入的数据
 * @return 无
 */
void OLED_Write_DATA(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (data & 0x80) { OLED_SDA_Set(); } else { OLED_SDA_Clr(); }
        OLED_SCL_Set();
        OLED_SCL_Clr();
        data <<= 1; // 每次循环后左移一位
    }
    OLED_SCL_Set();
    OLED_SCL_Clr();
}


/**
 * @brief OLED写数据
 * @param Data 要写入的数据数组
 * @param Count 要写入的数据个数
 */
void OLED_WriteDataArr(uint8_t *Data, uint8_t Count)
{
	OLED_I2C_Start();				//I2C起始
	OLED_Write_DATA(0x78);		//发送OLED的I2C从机地址
	OLED_Write_DATA(0x40);		//控制字节，给0x40，表示即将写数据
	if (OLED_ColorMode) {
        for (uint8_t i = 0; i < Count; i++) {
            OLED_Write_DATA(Data[i]);
        }
    } else {
        for (uint8_t i = 0; i < Count; i++) {
            OLED_Write_DATA(~Data[i]);
        }
    }
	OLED_I2C_Stop();				//I2C终止
}


/**
 * @brief OLED写命令
 * @param data 要写入的命令
 * @return 无
 */
void  OLED_Write_CMD(uint8_t data)
{	
	OLED_I2C_Start();				//I2C起始
	OLED_Write_DATA(0x78);		//发送OLED的I2C从机地址
	OLED_Write_DATA(0x00);		//控制字节，给0x00，表示即将写命令
	OLED_Write_DATA(data);		//写入指定的命令
	OLED_I2C_Stop();				//I2C终止	  
}




/**
 * @brief 设置显示光标位置
 * @param Page 页号
 * @param X X轴坐标
 * @return 无
 */
void OLED_SetCursor(uint8_t Page, uint8_t X)
{
	/*可以在此调整X，以适应一些芯片X轴坐标的偏移*/
	/*X += 2;*/
	/*通过指令设置页地址和列地址*/
    X += 28;
	OLED_Write_CMD(0xB0 | Page);					//设置页位置
	OLED_Write_CMD(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_Write_CMD(0x00 | (X & 0x0F));			//设置X位置低4位
}



/**
 * @brief 计算 CRC-16-CCITT 校验值
 * @param data 要计算校验值的字节数组
 * @param length 要计算校验值的字节长度
 * @return 校验值
 */
uint16_t compute_crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    const uint8_t *end = data + length;
    while (data < end) {
        crc ^= *data++ << 8;
        // 展开的位计算
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}


/**
 * @brief 计算动态刷新校验值
 * @note 计算动态刷新校验值，并更新历史校验值
 * @return 无
 */
void OLED_CalcVerify(void) {
    #if IF_ENABLE_DYNAMIC_REFRESH
    for (uint8_t page = 0; page < OLED_PAGE_DRIVER; page++) {
        // 计算每个页面分块的校验值
        for (uint8_t block = 0; block < (OLED_WIDTH_DRIVER / DYNAMIC_REFRESH_LENGHT); block++) {
            uint8_t xor_sum = 0;
            uint16_t crc_sum;
            uint16_t start_col = block * DYNAMIC_REFRESH_LENGHT;
            uint16_t end_col = start_col + DYNAMIC_REFRESH_LENGHT;

            // 更新历史校验值
            previous_page_checksum[page][block] = page_checksum[page][block];

            // 计算带列号混淆的 XOR
            for (uint16_t col = start_col; col < end_col; col++) {
                xor_sum ^= OLED_DisplayBuf[page][col] ^ (uint8_t)col + OLED_ColorMode;
            }

            // 计算改进版 CRC（仅计算当前块的数据）
            crc_sum = compute_crc16(&OLED_DisplayBuf[page][start_col], DYNAMIC_REFRESH_LENGHT);

            // 非对称组合校验值
            page_checksum[page][block] = crc_sum ^ (xor_sum << 8);
        }
    }
    #endif
}


/**
 * @brief 刷新显示
 * @note 刷新显示，如果开启了动态刷新，则仅刷新有变化的区域。
 * @return 无
 */
void OLED_Update(void)
{
    
    OLED_IfUpdate = false;
#if IF_ENABLE_DYNAMIC_REFRESH
	OLED_CalcVerify();
    /* 遍历每一页 */
    for (uint8_t page = 0; page < OLED_PAGE_DRIVER; page++)
    {
        /* 遍历每个块 */
        for (uint8_t block = 0; block < (OLED_WIDTH_DRIVER / DYNAMIC_REFRESH_LENGHT); block++)
        {
            /* 仅当该块有变化时才刷新 */
            if (page_checksum[page][block] != previous_page_checksum[page][block])
            {
                /* 计算当前块的起始列 */
                uint16_t start_col = block * DYNAMIC_REFRESH_LENGHT;
                /* 设置光标位置为该页的块起始列 */
                OLED_SetCursor(page, start_col);
                /* 将该块的数据写入 OLED */
                OLED_WriteDataArr(&OLED_DisplayBuf[page][start_col], DYNAMIC_REFRESH_LENGHT);
                OLED_IfUpdate = true;
            }
        }
    }
#else
    uint8_t page;
    for (page = 0; page < OLED_PAGE_DRIVER; page++)
    {
        /* 设置光标位置为每一页的第一列 */
        OLED_SetCursor(page, 0);
        /* 连续写入 OLED_WIDTH_DRIVER 个数据，将显存数组的数据写入到 OLED 硬件 */
        OLED_WriteDataArr(OLED_DisplayBuf[page], OLED_WIDTH_DRIVER);
    }
    OLED_IfUpdate = true;
#endif
}


/**
 * @brief 刷新指定区域
 * @param X X轴坐标
 * @param Y Y轴坐标
 * @param Width 宽度
 * @param Height 高度
 * @note 此函数会至少更新参数指定的区域。如果更新区域Y轴只包含部分页，则同一页的剩余部分会跟随一起更新
 * @return 无
 */
void OLED_UpdateArea(int16_t X, int16_t Y, int16_t Width, int16_t Height)
{
	int16_t j;
	
	/*参数检查，保证指定区域不会超出屏幕范围*/
	if (X > OLED_WIDTH_DRIVER-1) {return;}
	if (Y > OLED_HEIGHT_DRIVER-1) {return;}
	if (X < 0) {X = 0;}
	if (Y < 0) {Y = 0;}
	if (X + Width > OLED_WIDTH_DRIVER) {Width = OLED_WIDTH_DRIVER - X;}
	if (Y + Height > OLED_HEIGHT_DRIVER) {Height = OLED_HEIGHT_DRIVER - Y;}
	
	/*遍历指定区域涉及的相关页*/
	/*(Y + Height - 1) / 8 + 1的目的是(Y + Height) / 8并向上取整*/
	for (j = Y / 8; j < (Y + Height - 1) / 8 + 1; j ++)
	{
		/*设置光标位置为相关页的指定列*/
		OLED_SetCursor(j, X);
		/*连续写入Width个数据，将显存数组的数据写入到OLED硬件*/
		OLED_WriteDataArr(&OLED_DisplayBuf[j][X], Width);
	}
	
}
//声明一下OLED_Clear的清屏函数，以便在OLED_Init函数中调用
extern void OLED_Clear(void);

/**
 * @brief OLED初始化
 * @return 无
 */
void OLED_Init(void)
{
    OLED_DelayMs(100);	//等待100ms，等待OLED初始化完成
	/* 将SCL: PC1和SDA: PC2引脚初始化为开漏模式 */
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    OLED_SCL_Set();
    OLED_SDA_Set();

    OLED_Write_CMD(0xAE); /*关闭显示*/
    OLED_Write_CMD(0xD5); /*设置显示时钟分频比/振荡器频率*/
    OLED_Write_CMD(0xF0);
    OLED_Write_CMD(0xA8); /*设置多路复用率*/
    OLED_Write_CMD(0x27); /*设置占空比为1/40*/
    OLED_Write_CMD(0xD3); /*设置显示偏移*/
    OLED_Write_CMD(0x00);
    OLED_Write_CMD(0x40); /*设置显示起始行*/
    OLED_Write_CMD(0x8d); /*使能电荷泵*/
    OLED_Write_CMD(0x14);
    OLED_Write_CMD(0x20); /*设置内存地址模式*/
    OLED_Write_CMD(0x02); /*页地址模式*/
    OLED_Write_CMD(0xA1); /*段重映射设置*/
    OLED_Write_CMD(0xC8); /*COM扫描方向设置*/
    OLED_Write_CMD(0xDA); /*设置COM引脚配置*/
    OLED_Write_CMD(0x12);
    OLED_Write_CMD(0xAD); /*内部IREF设置*/
    OLED_Write_CMD(0x30);
    OLED_Write_CMD(0x81); /*对比度控制*/
    OLED_Write_CMD(0xff); /*设置对比度为最大值128*/
    OLED_Write_CMD(0xD9); /*设置预充电周期*/
    OLED_Write_CMD(0x22);
    OLED_Write_CMD(0xdb); /*设置VCOMH电压*/
    OLED_Write_CMD(0x20);
    OLED_Write_CMD(0xA4); /*全局显示开启/关闭设置*/
    OLED_Write_CMD(0xA6); /*正常/反转显示设置*/
    OLED_Write_CMD(0x0C); /*设置低列地址*/
    OLED_Write_CMD(0x11); /*设置高列地址*/

	OLED_Clear();
    
	OLED_Update();
    OLED_Write_CMD(0xAF);	//开启显示
	
}



/**
 * @brief 设置亮度
 * @param Brightness 亮度值，0-255
 * @note 一些屏幕芯片可能会在亮度较低的时候直接黑屏，需要注意一下。
 * @return 无
 */
void OLED_SetBrightness(int16_t Brightness){
	if(Brightness>255){
		Brightness=255;
	}
	if(Brightness<0){
		Brightness=0;
	}
	OLED_Write_CMD(0x81);
	OLED_Write_CMD(Brightness);
 }


