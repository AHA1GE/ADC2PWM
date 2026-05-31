/*========================================================================*/
/*========================================================================*/
// 这个头文件是OLED-Basic-Lib库的 [软件层] 实现文件，移植的时候只需要修改头文件的屏幕宽度与高度
// 版本v1.0.0
/*========================================================================*/
/*========================================================================*/
#include "OLED.h"
/**
  * 声明OLED显存数组，此数组已经在OLED_Driver.c中定义
  * 所有的显示函数，都只是对此显存数组进行读写
  * 随后调用OLED_Update函数或OLED_UpdateArea函数
  * 才会将显存数组的数据发送到OLED硬件，进行显示
  */
extern uint8_t OLED_DisplayBuf[OLED_HEIGHT/8][OLED_WIDTH];

#define OLED_ASCII 		    (0)
#define OLED_CHINESE 		(1)

/**关于字符串最大长度的宏，用于格式化输出字符串*/
#define  MAX_STRING_LENGTH   (128)

/**关于是否在绘制图像或是文字之前提前清除绘制区域显存的宏 */
#define IF_CLEAR_AREA        (true)


/*********************工具函数↓********************/

/**
 * @brief 计算X的Y次方
 * @param X 底数
 * @param Y 指数
 * @return X的Y次方
 */
inline uint32_t OLED_Pow(uint32_t X, uint32_t Y) {
    uint32_t result = 1;
    while (Y--) {
        result *= X;
    }
    return result;
}


/**
 * @brief 获取字符宽度
 * @param oledfont 字体类型
 * @param ascii_or_chinese 宏定义，表示是ASCII字符还是中文字符
 * @return 字符宽度
 */
uint8_t OLED_GetFontWidth(uint8_t oledfont, uint8_t ascii_or_chinese) {
    switch (oledfont) {
#ifdef OLED_FONT_8
        case OLED_FONT_8:  return (ascii_or_chinese == OLED_ASCII) ? 6 : 8;
#endif

#ifdef OLED_FONT_12
        case OLED_FONT_12: return (ascii_or_chinese == OLED_ASCII) ? 7 : 12;
#endif

#ifdef OLED_FONT_16
        case OLED_FONT_16: return (ascii_or_chinese == OLED_ASCII) ? 8 : 16;
#endif

#ifdef OLED_FONT_20
        case OLED_FONT_20: return (ascii_or_chinese == OLED_ASCII) ? 10 : 20;
#endif		
    }
	return 0;
}


/**
 * @brief 判断指定点是否在指定多边形内部
 * @param nvert 多边形的顶点数
 * @param vertx verty 包含多边形顶点的x和y坐标的数组
 * @param testx testy 测试点的X和y坐标
 * @return 指定点是否在指定多边形内部，1：在内部，0：不在内部
 */
uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
	int16_t i = 0, j = 0;
	uint8_t c = 0;
	/*此算法由W. Randolph Franklin提出*/
	/*参考链接：https://wrfranklin.org/Research/Short_Notes/pnpoly.html*/
	for (i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if (((verty[i] > testy) != (verty[j] > testy)) &&
			(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
		{
			c = !c;
		}
	}
	return c;
}

/**
 * @brief 比大小函数
 * @param a 第一个值
 * @param b 第二个值
 * @param c 第三个值
 * @param d 第四个值
 * @return 四个传入值当中的最大值
 */
int16_t max(int16_t a, int16_t b, int16_t c, int16_t d) {
    int16_t max_val = a; // 假设a是最大的

    if (b > max_val) {
        max_val = b; // 如果b大于当前最大值，则更新最大值为b
    }
    if (c > max_val) {
        max_val = c; // 如果c大于当前最大值，则更新最大值为c
    }
    if (d > max_val) {
        max_val = d; // 如果d大于当前最大值，则更新最大值为d
    }

    return max_val; // 返回最大值
}


/**
 * @brief 绝对值函数
 * @param num 输入值
 * @return num的绝对值
 */
float numabs(float num){
	if(num>0)
		return num;
	if(num<0)
		return -num;
	return 0;
}




/**
 * @brief 判断指定点是否在指定角度内部
 * @param X 指定点的X坐标
 * @param Y 指定点的Y坐标
 * @param StartAngle 起始角度
 * @param EndAngle 终止角度
 * @return 指定点是否在指定角度内部，1：在内部，0：不在内部
 */
uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle)
{
	int16_t PointAngle;
	PointAngle = atan2(Y, X) / 3.14 * 180;	//计算指定点的弧度，并转换为角度表示
	if (StartAngle < EndAngle)	//起始角度小于终止角度的情况
	{
		/*如果指定角度在起始终止角度之间，则判定指定点在指定角度*/
		if (PointAngle >= StartAngle && PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	else			//起始角度大于于终止角度的情况
	{
		/*如果指定角度大于起始角度或者小于终止角度，则判定指定点在指定角度*/
		if (PointAngle >= StartAngle || PointAngle <= EndAngle)
		{
			return 1;
		}
	}
	return 0;		//不满足以上条件，则判断判定指定点不在指定角度
}

/*********************工具函数↑********************/

/*********************功能函数↓*********************/

/**
 * @brief 清空显存数组
 * @param 无
 * @return 无
 */
void OLED_Clear(void) {
    memset(OLED_DisplayBuf, 0, sizeof(OLED_DisplayBuf));
}

/**
 * @brief 清空指定区域
 * @param X 指定区域左上角的横坐标，范围：随意，无效自动舍弃
 * @param Y 指定区域左上角的纵坐标，范围：随意，无效自动舍弃
 * @param Width 指定区域的宽度，范围：随意，无效自动舍弃
 * @param Height 指定区域的高度，范围：正数，无效自动舍弃
 * @return 无
 */
void OLED_ClearArea(int16_t X, int16_t Y, int16_t Width, int16_t Height) {
    if (Width <= 0 || Height <= 0) return;

    int16_t x_start = (X < 0) ? 0 : X;
    int16_t y_start = (Y < 0) ? 0 : Y;
    int16_t x_end = (X + Width > OLED_WIDTH) ? OLED_WIDTH : (X + Width);
    int16_t y_end = (Y + Height > OLED_HEIGHT) ? OLED_HEIGHT : (Y + Height);

    if (x_start >= x_end || y_start >= y_end) return;

    int16_t start_page = y_start >> 3;
    int16_t end_page = (y_end - 1) >> 3;
    uint8_t start_mask = 0xFF << (y_start & 7);
    uint8_t end_mask = 0xFF >> (7 - ((y_end - 1) & 7));

    for (int16_t page = start_page; page <= end_page; page++) {
        uint8_t mask = 0xFF;
        if (page == start_page) mask &= start_mask;
        if (page == end_page) mask &= end_mask;
        for (int16_t x = x_start; x < x_end; x++) {
            OLED_DisplayBuf[page][x] &= ~mask;
        }
    }
}

/**
 * @brief 反色显存数组
 * @param 无
 * @return 无
 */
void OLED_Reverse(void)
{
	uint16_t i, j;
	for (j = 0; j < OLED_HEIGHT/8; j ++)				//遍历页
	{
		for (i = 0; i < OLED_WIDTH; i ++)			//遍历OLED_WIDTH列
		{
			OLED_DisplayBuf[j][i] ^= 0xFF;	//将显存数组数据全部取反
		}
	}
}

/**
 * @brief 反色指定区域
 * @param X 指定区域左上角的横坐标，范围：随意，无效自动舍弃
 * @param Y 指定区域左上角的纵坐标，范围：随意，无效自动舍弃
 * @param Width 指定区域的宽度，范围：随意，无效自动舍弃
 * @param Height 指定区域的高度，范围：正数，无效自动舍弃
 * @return 无
 */
void OLED_ReverseArea(int16_t X, int16_t Y, int16_t Width, int16_t Height)
{
	int16_t i, j, x, y;
	if(Width <= 0 || Height <= 0) {return; }
	/*参数检查，保证指定区域不会超出屏幕范围*/
	if (X > OLED_WIDTH-1) {return;}
	if (Y > OLED_HEIGHT-1) {return;}
	if (X + Width > OLED_WIDTH) {Width = OLED_WIDTH - X;}
	if (Y + Height > OLED_HEIGHT) {Height = OLED_HEIGHT - Y;}
	if (X + Width < 0) {return;}
	if (Y + Height < 0) {return;}
	if (X < 0) { x = 0;} else { x = X;}
	if (Y < 0) { y = 0;} else { y = Y;}
	
	for (j = y; j < Y + Height; j ++)		//遍历指定页
	{
		for (i = x; i < X + Width; i ++)	//遍历指定列
		{
			OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8);	//将显存数组指定数据取反
		}
	}
}
/**
  * @brief 在指定区域内显示图片
  * @param X_Pic 图片左上角的横坐标
  * @param Y_Pic 图片左上角的纵坐标
  * @param PictureWidth 图片宽度
  * @param PictureHeight 图片高度
  * @param X_Area 显示区域的左上角的横坐标
  * @param Y_Area 显示区域的左上角的纵坐标
  * @param AreaWidth 显示区域的宽度
  * @param AreaHeight 显示区域的高度
  * @param Image 图片取模数组
  * @note 此函数至关重要，它可以将一个图片显示在指定的区域内，实现复杂的显示效果，为OLED_UI的诸多功能提供基础。
  * @retval 无
  */
 void OLED_ShowImageArea(int16_t X_Area, int16_t Y_Area, int16_t AreaWidth, int16_t AreaHeight, int16_t X_Pic, int16_t Y_Pic, int16_t PictureWidth, int16_t PictureHeight, const uint8_t *Image)
 {
	 if (PictureWidth == 0 || PictureHeight == 0 || AreaWidth == 0 || AreaHeight == 0 || X_Pic > OLED_WIDTH-1 || X_Area > OLED_WIDTH-1 || Y_Pic > OLED_HEIGHT-1 || Y_Area > OLED_HEIGHT-1) {return; }
		  int16_t startX = (X_Pic < X_Area) ? X_Area : X_Pic;
	 int16_t endX = ((X_Area + AreaWidth - 1) < (X_Pic + PictureWidth - 1)) ? (X_Area + AreaWidth - 1) : (X_Pic + PictureWidth - 1);
	 int16_t startY = (Y_Pic < Y_Area) ? Y_Area : Y_Pic;
	 int16_t endY = ((Y_Area + AreaHeight - 1) < (Y_Pic + PictureHeight - 1)) ? (Y_Area + AreaHeight - 1) : (Y_Pic + PictureHeight - 1);
	 endX = (endX > OLED_WIDTH-1) ? OLED_WIDTH-1 : endX;
	 endY = (endY > OLED_HEIGHT-1) ? OLED_HEIGHT-1 : endY;
		 if(startX > endX || startY > endY){return;}
#if IF_CLEAR_AREA
		 OLED_ClearArea(startX, startY, endX - startX + 1, endY - startY + 1);
#endif
		 for (uint8_t j = 0; j <= (PictureHeight - 1) / 8; j++) {
		 for (uint8_t i = 0; i < PictureWidth; i++) {
			 uint8_t currX = X_Pic + i;
			 if (currX < startX || currX > endX) {continue;};
			 for (uint8_t bit = 0; bit < 8; bit++) {
				 uint8_t currY = Y_Pic + j * 8 + bit;
				 if (currY < startY || currY > endY) {continue;};
				 uint8_t page = currY / 8;
				 uint8_t bit_pos = currY % 8;
				 uint8_t data = Image[j * PictureWidth + i];
				 if (data & (1 << bit)) {OLED_DisplayBuf[page][currX] |= (1 << bit_pos); }
			 }
		 }
	 }
 }
 
 
 
/**
 * @brief 显示图像
 * @param X 指定图像左上角的横坐标，范围：随意，无效自动舍弃
 * @param Y 指定图像左上角的纵坐标，范围：范围：随意，无效自动舍弃
 * @param Width 指定图像的宽度，范围：正数
 * @param Height 指定图像的高度，范围：正数
 * @param Image 指定要显示的图像
 * @return 无
 */
void OLED_ShowImage(int16_t X, int16_t Y, uint16_t Width, uint16_t Height, const uint8_t *Image)
{
    OLED_ShowImageArea(0, 0, OLED_WIDTH, OLED_HEIGHT, X, Y, Width, Height, Image);
}

/**
  * @brief 在指定范围内显示一个字符
  * @param RangeX 指定字符可以显示范围的左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param RangeY 指定字符可以显示范围的左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param RangeWidth 指定范围宽度
  * @param RangeHeight 指定范围高度
  * @param X 指定字符左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param Y 指定字符左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param Char 指定要显示的字符，范围：ASCII码可见字符
  * @param FontSize 指定字体大小,OLED_6X8_HALF,OLED_7X12_HALF,OLED_8X16_HALF,OLED_10X20_HALF
  * @retval 无
  */
void OLED_ShowCharArea(int16_t RangeX, int16_t RangeY, int16_t RangeWidth, int16_t RangeHeight, 
                       int16_t X, int16_t Y, char Char, uint8_t FontSize) {
    if (Char < ' ' || Char > '~') {
        // 如果传入的字符不在可打印范围内，直接返回
        return;
    }

                        int16_t width = 0, height = 0;
                        const uint8_t *font_data = NULL;

    switch (FontSize) {
		#ifdef OLED_FONT_8
        case OLED_FONT_8:
            width = 6;
            height = 8;
            font_data = OLED_F6x8[Char - ' '];
            break;

		#endif

		#ifdef OLED_FONT_12
        case OLED_FONT_12:
            width = 7;
            height = 12;
            font_data = OLED_F7x12[Char - ' '];
            break;
		#endif

		#ifdef OLED_FONT_16
        case OLED_FONT_16:
            width = 8;
            height = 16;
            font_data = OLED_F8x16[Char - ' '];
            break;
		#endif

		#ifdef OLED_FONT_20
        case OLED_FONT_20:
            width = 10;
            height = 20;
            font_data = OLED_F10x20[Char - ' '];
            break;
		#endif
			
    }
                        if (font_data != NULL)
                            OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, width, height, font_data);

    OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, width, height, font_data);
}


/**
  * @brief 在指定区域范围内OLED显示混合字符串（汉字与ASCII）
  * @param RangeX 指定字符可以显示范围的左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param RangeY 指定字符可以显示范围的左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param RangeWidth 指定范围宽度
  * @param RangeHeight 指定范围高度
  * @param X 指定字符左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param Y 指定字符左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param String 指定要显示的混合字符串，范围：全角字符与半角字符都可以
  *           显示的汉字需要在OLED_Data.c里的OLED_CF16x16或OLED_CF12x12数组定义
  *           未找到指定汉字时，会显示默认图形（一个方框，内部一个问号）
  * @param ChineseFontSize 指定中文文字大小，OLED_12X12或OLED_16X16或OLED_8X8
  * @param ASCIIFontSize  指定ASCII文字大小,OLED_6X8或OLED_7X12或OLED_8X16
  * @retval 无
  */

void OLED_ShowMixStringArea(int16_t RangeX, int16_t RangeY, int16_t RangeWidth, int16_t RangeHeight, 
                           int16_t X, int16_t Y, const char *String, uint8_t Font)
{
    int16_t originX = X;  // 记录行首X坐标
    
    while (*String != '\0') {
        // 处理换行符
        if (*String == '\n') {
            X = originX;  // 返回行首
            Y += Font + OLED_LINE_SPACING;  // 移动到下一行
            String++;
            
            continue;
        }

        if (*String & 0x80) {  // 中文字符处理
            // char Chinese[OLED_CHN_CHAR_WIDTH + 1];
            // for (uint8_t i = 0; i < OLED_CHN_CHAR_WIDTH; i++) {
            //     Chinese[i] = *(String + i);
            // }
            // Chinese[OLED_CHN_CHAR_WIDTH] = '\0';
            // OLED_ShowChineseArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, Chinese, Font);
            // X += OLED_GetFontWidth(Font, OLED_CHINESE) + OLED_CHAR_SPACING;
            // String += OLED_CHN_CHAR_WIDTH;
        } else {  // ASCII字符处理
            OLED_ShowCharArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, *String, Font);
            X += OLED_GetFontWidth(Font, OLED_ASCII) + OLED_CHAR_SPACING;
            String++;
        }

    }
}

/**
 * @brief 画点
 * @param X 点的横坐标
 * @param Y 点的纵坐标
 * @return 无
 */
inline void OLED_DrawPoint(int16_t X, int16_t Y) {
    if (X < 0 || Y < 0 || X >= OLED_WIDTH || Y >= OLED_HEIGHT) return;
    OLED_DisplayBuf[Y >> 3][X] |= (1 << (Y & 7));
}

/**
 * @brief 获取指定位置点的值
 * @param X 点的横坐标
 * @param Y 点的纵坐标
 * @return 1：点亮，0：熄灭
 */
inline uint8_t OLED_GetPoint(int16_t X, int16_t Y) {
    if (X < 0 || Y < 0 || X >= OLED_WIDTH || Y >= OLED_HEIGHT) return 0;
    return (OLED_DisplayBuf[Y >> 3][X] >> (Y & 7)) & 1;
}
