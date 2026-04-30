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
 * @brief 将整数转换为字符串
 * @param num 要转换的整数
 * @param str 输出缓冲区
 * @return 输出字符串
 */
static char* itoa(int num, char* str, int base) {
    char* ptr = str;
    char temp_buf[33]; // 增加缓冲区大小，确保安全
    int temp_ptr = 0;
    int is_negative = 0;
    unsigned int value;
    
    // 处理零
    if (num == 0) {
        *ptr++ = '0';
        *ptr = '\0';
        return str;
    }
    
    // 处理负数
    if (num < 0 && base == 10) {
        is_negative = 1;
        value = -num;
    } else {
        // 非十进制或正数，直接转为无符号处理
        value = (unsigned int)num;
    }
    
    // 转换数字到字符，存入临时缓冲区
    while (value > 0) {
        int digit = value % base;
        temp_buf[temp_ptr++] = (digit < 10) ? (digit + '0') : (digit - 10 + 'a');
        value /= base;
    }
    
    // 如果是负数，添加负号
    if (is_negative) {
        *ptr++ = '-';
    }
    
    // 反向复制临时缓冲区到结果字符串
    while (temp_ptr > 0) {
        *ptr++ = temp_buf[--temp_ptr];
    }
    
    *ptr = '\0';
    return str;
}

/**
 * @brief 将浮点数转换为字符串
 * @param num 要转换的浮点数
 * @param str 输出缓冲区
 * @param precision 精度，小数点后几位
 * @return 输出字符串
 */
static char* ftoa(float num, char* str, int precision) {
    char* ptr = str;
    int int_part;
    float float_part;
    int i;
    char temp_buf[32]; // 用于临时存储整数部分
    int temp_ptr = 0;
    
    // 处理负数 - 先记录符号，最后再添加
    int is_negative = 0;
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }
    
    // 分离整数部分和小数部分
    int_part = (int)num;
    float_part = num - int_part;
    
    // 如果是负数，先添加负号
    if (is_negative) {
        *ptr++ = '-';
    }
    
    // 转换整数部分 - 先存入临时缓冲区
    if (int_part == 0) {
        *ptr++ = '0';
    } else {
        while (int_part > 0) {
            temp_buf[temp_ptr++] = '0' + (int_part % 10);
            int_part /= 10;
        }
        
        // 反转并复制到结果字符串
        for (i = temp_ptr - 1; i >= 0; i--) {
            *ptr++ = temp_buf[i];
        }
    }
    
    // 处理小数部分
    if (precision > 0) {
        *ptr++ = '.';
        
        // 计算小数部分
        for (i = 0; i < precision; i++) {
            float_part *= 10;
            int digit = (int)float_part;
            *ptr++ = '0' + digit;
            float_part -= digit;
        }
    }
    
    *ptr = '\0';
    return str;
}

/**
 * @brief 一个简单的sprintf函数，用于将格式化字符串输出到缓冲区
 * @param buf 输出缓冲区
 * @param format 格式化字符串
 * @param... 格式化参数
 * @return 写入的字符数
 */
int OLED_Simple_vsprintf(char* buf, const char* format, va_list args) {
    char* str_ptr = buf;
    const char* fmt_ptr = format;
    char temp_buf[32];  // 足够容纳大多数数字转换
    char padding_char;
    int padding_width, precision;
    int left_align; // 新增：左对齐标志
    int len, i;
    
    while (*fmt_ptr) {
        // 非格式化字符直接复制
        if (*fmt_ptr != '%') {
            *str_ptr++ = *fmt_ptr++;
            continue;
        }
        
        fmt_ptr++;  // 跳过 '%'
        
        // 处理 '%%' 转义
        if (*fmt_ptr == '%') {
            *str_ptr++ = '%';
            fmt_ptr++;
            continue;
        }
        
        // 解析填充规则和宽度
        padding_char = ' ';
        padding_width = 0;
        precision = 6;  // 默认浮点数精度
        left_align = 0; // 默认右对齐
        
        // 检查左对齐标志
        if (*fmt_ptr == '-') {
            left_align = 1;
            fmt_ptr++;
        }
        
        if (*fmt_ptr == '0' && !left_align) { // 0填充仅在非左对齐时有效
            padding_char = '0';
            fmt_ptr++;
        }
        
        // 解析宽度
        while (*fmt_ptr >= '0' && *fmt_ptr <= '9') {
            padding_width = padding_width * 10 + (*fmt_ptr - '0');
            fmt_ptr++;
        }
        
        // 解析精度
        if (*fmt_ptr == '.') {
            fmt_ptr++;
            precision = 0;
            while (*fmt_ptr >= '0' && *fmt_ptr <= '9') {
                precision = precision * 10 + (*fmt_ptr - '0');
                fmt_ptr++;
            }
        }
        
        // 根据格式说明符处理参数
        switch (*fmt_ptr) {
            case 'd': 
            case 'i': {
                int val = va_arg(args, int);
                itoa(val, temp_buf, 10);
                len = 0;
                while (temp_buf[len]) len++;
                
                // 处理填充
                if (!left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = padding_char;
                    }
                }
                
                // 拷贝数字
                for (i = 0; i < len; i++) {
                    *str_ptr++ = temp_buf[i];
                }
                
                // 左对齐时在右侧填充
                if (left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = ' '; // 左对齐时右侧始终用空格填充
                    }
                }
                break;
            }
            case 'u': {
                unsigned int val = va_arg(args, unsigned int);
                // 将无符号整数转换为字符串 (使用itoa并传递正数)
                itoa(val, temp_buf, 10);
                len = 0;
                while (temp_buf[len]) len++;
                
                // 处理填充和输出 (与%d类似)
                if (!left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = padding_char;
                    }
                }
                
                for (i = 0; i < len; i++) {
                    *str_ptr++ = temp_buf[i];
                }
                
                if (left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = ' ';
                    }
                }
                break;
            }
            case 'f': {
                float val = (float)va_arg(args, double);  // va_arg 对 float 参数使用 double
                ftoa(val, temp_buf, precision);
                len = 0;
                while (temp_buf[len]) len++;
                
                // 处理填充
                if (!left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = padding_char;
                    }
                }
                
                // 拷贝数字
                for (i = 0; i < len; i++) {
                    *str_ptr++ = temp_buf[i];
                }
                
                if (left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = ' ';
                    }
                }
                break;
            }
            case 'c': {
                char val = (char)va_arg(args, int);  // char 在 va_arg 中提升为 int
                
                // 处理填充
                if (!left_align) {
                    for (i = 0; i < padding_width - 1; i++) {
                        *str_ptr++ = padding_char;
                    }
                }
                
                *str_ptr++ = val;
                
                if (left_align) {
                    for (i = 0; i < padding_width - 1; i++) {
                        *str_ptr++ = ' ';
                    }
                }
                break;
            }
            case 's': {
                char* val = va_arg(args, char*);
                len = 0;
                while (val[len]) len++;
                
                // 处理填充
                if (!left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = padding_char;
                    }
                }
                
                // 拷贝字符串
                for (i = 0; i < len; i++) {
                    *str_ptr++ = val[i];
                }
                
                if (left_align) {
                    for (i = 0; i < padding_width - len; i++) {
                        *str_ptr++ = ' ';
                    }
                }
                break;
            }
            default:
                // 未知格式，忽略
                break;
        }
        
        fmt_ptr++;
    }
    
    *str_ptr = '\0';  // 确保字符串以 null 结尾
    return str_ptr - buf;  // 返回写入的字符数
}







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

    int16_t width, height;
    const uint8_t *font_data;

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

    OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, width, height, font_data);
}


 

/**
  * @brief 在指定区域范围内OLED显示汉字串
  * @param RangeX 指定字符可以显示范围的左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param RangeY 指定字符可以显示范围的左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param RangeWidth 指定范围宽度
  * @param RangeHeight 指定范围高度
  * @param X 指定字符左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param Y 指定字符左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param Chinese 指定要显示的汉字串，范围：必须全部为汉字或者全角字符，不要加入任何半角字符
  *           显示的汉字需要在OLED_Data.c里的OLED_CF16x16或OLED_CF12x12数组定义
  *           未找到指定汉字时，会显示默认图形（一个方框，内部一个问号）
  * @param FontSize 指定中文文字大小，，OLED_8X8_FULL,OLED_12X12_FULL,OLED_16X16_FULL,OLED_20X20_FULL
  * @retval 无
  */


void OLED_ShowChineseArea(int16_t RangeX,int16_t RangeY,int16_t RangeWidth,int16_t RangeHeight, int16_t X, int16_t Y, char *Chinese, uint8_t FontSize)
{
    uint8_t pChinese = 0;
    uint8_t pIndex;
    uint8_t i;
    char SingleChinese[OLED_CHN_CHAR_WIDTH + 1] = {0};
    for (i = 0; Chinese[i] != '\0'; i ++)    // 遍历汉字串
    {
        SingleChinese[pChinese] = Chinese[i];    // 提取汉字串数据到单个汉字数组
        pChinese ++;                            // 计次自增
        
        if (pChinese >= OLED_CHN_CHAR_WIDTH)    // 提取到了一个完整的汉字
        {
            pChinese = 0;    // 计次归零
            const void* fontArray;
			switch (FontSize) {
				#ifdef OLED_FONT_8
				case OLED_FONT_8:
					fontArray = (const void*) OLED_CF8x8;
					for (pIndex = 0; strcmp(((const ChineseCell8x8_t*)fontArray)[pIndex].Index, "") != 0; pIndex++) {
						if (strcmp(((const ChineseCell8x8_t*)fontArray)[pIndex].Index, SingleChinese) == 0) break;
					}
					OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, 
									  X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 8, Y, 8, 8, 
									  ((const ChineseCell8x8_t*)fontArray)[pIndex].Data);
					break;
				#endif
			
				#ifdef OLED_FONT_12
				case OLED_FONT_12:
					fontArray = (const void*) OLED_CF12x12;
					for (pIndex = 0; strcmp(((const ChineseCell12x12_t*)fontArray)[pIndex].Index, "") != 0; pIndex++) {
						if (strcmp(((const ChineseCell12x12_t*)fontArray)[pIndex].Index, SingleChinese) == 0) break;
					}
					OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, 
									  X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 12, Y, 12, 12, 
									  ((const ChineseCell12x12_t*)fontArray)[pIndex].Data);
					break;
				#endif
			
				#ifdef OLED_FONT_16
				case OLED_FONT_16:
					fontArray = (const void*) OLED_CF16x16;
					for (pIndex = 0; strcmp(((const ChineseCell16x16_t*)fontArray)[pIndex].Index, "") != 0; pIndex++) {
						if (strcmp(((const ChineseCell16x16_t*)fontArray)[pIndex].Index, SingleChinese) == 0) break;
					}
					OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, 
									  X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 16, Y, 16, 16, 
									  ((const ChineseCell16x16_t*)fontArray)[pIndex].Data);
					break;
				#endif
			
				#ifdef OLED_FONT_20
				case OLED_FONT_20:
					fontArray = (const void*) OLED_CF20x20;
					for (pIndex = 0; strcmp(((const ChineseCell20x20_t*)fontArray)[pIndex].Index, "") != 0; pIndex++) {
						if (strcmp(((const ChineseCell20x20_t*)fontArray)[pIndex].Index, SingleChinese) == 0) break;
					}
					OLED_ShowImageArea(RangeX, RangeY, RangeWidth, RangeHeight, 
									  X + ((i + 1) / OLED_CHN_CHAR_WIDTH - 1) * 20, Y, 20, 20, 
									  ((const ChineseCell20x20_t*)fontArray)[pIndex].Data);
					break;
				#endif
			}
			
			}
		}
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
                           int16_t X, int16_t Y, char *String, uint8_t Font)
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
            char Chinese[OLED_CHN_CHAR_WIDTH + 1];
            for (uint8_t i = 0; i < OLED_CHN_CHAR_WIDTH; i++) {
                Chinese[i] = *(String + i);
            }
            Chinese[OLED_CHN_CHAR_WIDTH] = '\0';
            OLED_ShowChineseArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, Chinese, Font);
            X += OLED_GetFontWidth(Font, OLED_CHINESE) + OLED_CHAR_SPACING;
            String += OLED_CHN_CHAR_WIDTH;
        } else {  // ASCII字符处理
            OLED_ShowCharArea(RangeX, RangeY, RangeWidth, RangeHeight, X, Y, *String, Font);
            X += OLED_GetFontWidth(Font, OLED_ASCII) + OLED_CHAR_SPACING;
            String++;
        }
    }
}




/**
  * 函    数：OLED使用printf函数打印格式化字符串,可以是中英文混杂的字符串。此函数由bilibili@上nm网课呢 添加
  * 参    数：X 指定汉字串左上角的横坐标，范围：负值~OLED_WIDTH-1
  * 参    数：Y 指定汉字串左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * 参    数：ChineseFontSize 指定汉字字体大小，OLED_8X8_FULL,OLED_12X12_FULL,OLED_16X16_FULL,OLED_20X20_FULL
  * 参    数：ASCIIFontSize 指定ASCII字体大小,OLED_6X8_HALF,OLED_7X12_HALF,OLED_8X16_HALF,OLED_10X20_HALF
  * 参    数：format 指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
  * 参    数：... 格式化字符串参数列表
  * 返 回 值：无
  * 说    明：调用此函数后，要想真正地呈现在屏幕上，还需调用更新函数
  */

void OLED_Printf(int16_t X, int16_t Y, uint8_t Font,const char *format, ...)
{
	char String[MAX_STRING_LENGTH];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	#if USE_SIMPLE_VSPRINTF
	OLED_Simple_vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	#else
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	#endif
	va_end(arg);							//结束变量arg
	OLED_ShowMixStringArea(0, 0, OLED_WIDTH, OLED_HEIGHT, X, Y, String, Font);//OLED显示字符数组（字符串）
}















/**
  * @brief OLED使用printf函数在指定区域内打印格式化字符串，此函数由bilibili@上nm网课呢 添加
  * @param RangeX 指定字符可以显示范围的左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param RangeY 指定字符可以显示范围的左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param RangeWidth 指定范围宽度
  * @param RangeHeight 指定范围高度
  * @param X 指定字符左上角的横坐标，范围：负值~OLED_WIDTH-1
  * @param Y 指定字符左上角的纵坐标，范围：负值~OLED_HEIGHT-1
  * @param ChineseFontSize 指定汉字字体大小,OLED_8X8_FULL,OLED_12X12_FULL,OLED_16X16_FULL,OLED_20X20_FULL
  * @param ASCIIFontSize 指定ASCII字体大小,OLED_6X8_HALF,OLED_7X12_HALF,OLED_8X16_HALF,OLED_10X20_HALF
			format 指定要显示的格式化字符串，范围：ASCII码可见字符组成的字符串
  * @param ... 格式化字符串参数列表
  * @return 无
  */

void OLED_PrintfArea(int16_t RangeX,int16_t RangeY,int16_t RangeWidth,int16_t RangeHeight,int16_t X, int16_t Y, uint8_t Font, char *format, ...)
{
	//由于有可能显示极长的字符串，所以128
	char String[MAX_STRING_LENGTH];						//定义字符数组
	va_list arg;							//定义可变参数列表数据类型的变量arg
	va_start(arg, format);					//从format开始，接收参数列表到arg变量
	#if USE_SIMPLE_VSPRINTF
	OLED_Simple_vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	#else
	vsprintf(String, format, arg);			//使用vsprintf打印格式化字符串和参数列表到字符数组中
	#endif
	va_end(arg);							//结束变量arg
	OLED_ShowMixStringArea(RangeX,RangeY,RangeWidth,RangeHeight,X, Y, String, Font);//OLED显示字符数组（字符串）
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




/**
 * @brief 画线
 * @param X0 线的起点横坐标
 * @param Y0 线的起点纵坐标
 * @param X1 线的终点横坐标
 * @param Y1 线的终点纵坐标
 * @return 无
 */
void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
	int16_t x, y, dx, dy, d, incrE, incrNE, temp;
	int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
	uint8_t yflag = 0, xyflag = 0;
	
	if (y0 == y1)		//横线单独处理
	{
		/*0号点X坐标大于1号点X坐标，则交换两点X坐标*/
		if (x0 > x1) {temp = x0; x0 = x1; x1 = temp;}
		
		/*遍历X坐标*/
		for (x = x0; x <= x1; x ++)
		{
			OLED_DrawPoint(x, y0);	//依次画点
		}
	}
	else if (x0 == x1)	//竖线单独处理
	{
		/*0号点Y坐标大于1号点Y坐标，则交换两点Y坐标*/
		if (y0 > y1) {temp = y0; y0 = y1; y1 = temp;}
		
		/*遍历Y坐标*/
		for (y = y0; y <= y1; y ++)
		{
			OLED_DrawPoint(x0, y);	//依次画点
		}
	}
	else				//斜线
	{
		/*使用Bresenham算法画直线，可以避免耗时的浮点运算，效率更高*/
		/*参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf*/
		/*参考教程：https://www.bilibili.com/video/BV1364y1d7Lo*/
		
		if (x0 > x1)	//0号点X坐标大于1号点X坐标
		{
			/*交换两点坐标*/
			/*交换后不影响画线，但是画线方向由第一、二、三、四象限变为第一、四象限*/
			temp = x0; x0 = x1; x1 = temp;
			temp = y0; y0 = y1; y1 = temp;
		}
		
		if (y0 > y1)	//0号点Y坐标大于1号点Y坐标
		{
			/*将Y坐标取负*/
			/*取负后影响画线，但是画线方向由第一、四象限变为第一象限*/
			y0 = -y0;
			y1 = -y1;
			
			/*置标志位yflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			yflag = 1;
		}
		
		if (y1 - y0 > x1 - x0)	//画线斜率大于1
		{
			/*将X坐标与Y坐标互换*/
			/*互换后影响画线，但是画线方向由第一象限0~90度范围变为第一象限0~45度范围*/
			temp = x0; x0 = y0; y0 = temp;
			temp = x1; x1 = y1; y1 = temp;
			
			/*置标志位xyflag，记住当前变换，在后续实际画线时，再将坐标换回来*/
			xyflag = 1;
		}
		
		/*以下为Bresenham算法画直线*/
		/*算法要求，画线方向必须为第一象限0~45度范围*/
		dx = x1 - x0;
		dy = y1 - y0;
		incrE = 2 * dy;
		incrNE = 2 * (dy - dx);
		d = 2 * dy - dx;
		x = x0;
		y = y0;
		
		/*画起始点，同时判断标志位，将坐标换回来*/
		if (yflag && xyflag){OLED_DrawPoint(y, -x);}
		else if (yflag)		{OLED_DrawPoint(x, -y);}
		else if (xyflag)	{OLED_DrawPoint(y, x);}
		else				{OLED_DrawPoint(x, y);}
		
		while (x < x1)		//遍历X轴的每个点
		{
			x ++;
			if (d < 0)		//下一个点在当前点东方
			{
				d += incrE;
			}
			else			//下一个点在当前点东北方
			{
				y ++;
				d += incrNE;
			}
			
			/*画每一个点，同时判断标志位，将坐标换回来*/
			if (yflag && xyflag){OLED_DrawPoint(y, -x);}
			else if (yflag)		{OLED_DrawPoint(x, -y);}
			else if (xyflag)	{OLED_DrawPoint(y, x);}
			else				{OLED_DrawPoint(x, y);}
		}	
	}
}


/**
 * @brief 画矩形
 * @param X 矩形左上角的横坐标
 * @param Y 矩形左上角的纵坐标
 * @param Width 矩形的宽度
 * @param Height 矩形的高度
 * @param IsFilled 是否填充
 * @return 无
 */

void OLED_DrawRectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, uint8_t IsFilled)
{
    if (Width == 0 || Height == 0) { return; } // 宽度或高度为0，不绘制

    int16_t X_start = X;
    int16_t X_end = X + Width - 1;
    int16_t Y_start = Y;
    int16_t Y_end = Y + Height - 1;

    // 限制坐标在屏幕范围内
    if (X_start < 0) X_start = 0;
    if (X_end >= OLED_WIDTH) X_end = OLED_WIDTH - 1;
    if (Y_start < 0) Y_start = 0;
    if (Y_end >= OLED_HEIGHT) Y_end = OLED_HEIGHT - 1;

    // 计算有效宽度和高度
    int16_t validWidth = X_end - X_start + 1;
    int16_t validHeight = Y_end - Y_start + 1;
    if (validWidth <= 0 || validHeight <= 0) { return; }

    
    // 计算起始和结束页
    int16_t start_page = Y_start / 8;
    int16_t end_page = Y_end / 8;

    // 计算每页的掩码
    uint8_t start_mask = 0xFF << (Y_start % 8);
    uint8_t end_mask = 0xFF >> (7 - (Y_end % 8));

    
		if(IsFilled){
			// 遍历每一列，应用掩码
			for (int16_t x = X_start; x <= X_end; x++) {
					for (int16_t page = start_page; page <= end_page; page++) {
							uint8_t mask = 0xFF;
							if (page == start_page) mask &= start_mask;
							if (page == end_page) mask &= end_mask;
							if (page >= 0 && page < OLED_HEIGHT / 8) { // 确保页数有效
									OLED_DisplayBuf[page][x] |= mask;
							}
					}
			}
		}else{
			// 绘制矩形边框
        for (int16_t i = X; i <= X + Width - 1; i++) {
            OLED_DrawPoint(i, Y);
            OLED_DrawPoint(i, Y + Height - 1);
        }
        for (int16_t i = Y; i <= Y + Height - 1; i++) {
            OLED_DrawPoint(X, i);
            OLED_DrawPoint(X + Width - 1, i);
        }
		}
		
		
   
}

/**
 * @brief 画三角形
 * @param X0 第一个端点的横坐标
 * @param Y0 第一个端点的纵坐标
 * @param X1 第二个端点的横坐标
 * @param Y1 第二个端点的纵坐标
 * @param X2 第三个端点的横坐标
 * @param Y2 第三个端点的纵坐标
 * @param IsFilled 是否填充
 * @return 无
 */
void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t IsFilled)
{
	int16_t minx = X0, miny = Y0, maxx = X0, maxy = Y0;
	int16_t i, j;
	int16_t vx[] = {X0, X1, X2};
	int16_t vy[] = {Y0, Y1, Y2};
	
	if (!IsFilled)			//指定三角形不填充
	{
		/*调用画线函数，将三个点用直线连接*/
		OLED_DrawLine(X0, Y0, X1, Y1);
		OLED_DrawLine(X0, Y0, X2, Y2);
		OLED_DrawLine(X1, Y1, X2, Y2);
	}
	else					//指定三角形填充
	{
		/*找到三个点最小的X、Y坐标*/
		if (X1 < minx) {minx = X1;}
		if (X2 < minx) {minx = X2;}
		if (Y1 < miny) {miny = Y1;}
		if (Y2 < miny) {miny = Y2;}
		
		/*找到三个点最大的X、Y坐标*/
		if (X1 > maxx) {maxx = X1;}
		if (X2 > maxx) {maxx = X2;}
		if (Y1 > maxy) {maxy = Y1;}
		if (Y2 > maxy) {maxy = Y2;}
		
		/*最小最大坐标之间的矩形为可能需要填充的区域*/
		/*遍历此区域中所有的点*/
		/*遍历X坐标*/
		for (i = minx; i <= maxx; i ++)
		{
			/*遍历Y坐标*/
			for (j = miny; j <= maxy; j ++)
			{
				/*调用OLED_pnpoly，判断指定点是否在指定三角形之中*/
				/*如果在，则画点，如果不在，则不做处理*/
				if (OLED_pnpoly(3, vx, vy, i, j)) {OLED_DrawPoint(i, j);}
			}
		}
	}
}

/**
 * @brief 画圆
 * @param X 圆心的横坐标
 * @param Y 圆心的纵坐标
 * @param Radius 圆的半径
 * @param IsFilled 是否填充
 * @return 无
 */
void OLED_DrawCircle(int16_t X, int16_t Y, int16_t Radius, uint8_t IsFilled)
{
	int16_t x, y, d, j;
	
	/*使用Bresenham算法画圆，可以避免耗时的浮点运算，效率更高*/
	/*参考文档：https://www.cs.montana.edu/courses/spring2009/425/dslectures/Bresenham.pdf*/
	/*参考教程：https://www.bilibili.com/video/BV1VM4y1u7wJ*/
	
	d = 1 - Radius;
	x = 0;
	y = Radius;
	
	/*画每个八分之一圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X + y, Y + x);
	OLED_DrawPoint(X - y, Y - x);
	
	if (IsFilled)		//指定圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在指定区域画点，填充部分圆*/
			OLED_DrawPoint(X, Y + j);
		}
	}
	
	while (x < y)		//遍历X轴的每个点
	{
		x ++;
		if (d < 0)		//下一个点在当前点东方
		{
			d += 2 * x + 1;
		}
		else			//下一个点在当前点东南方
		{
			y --;
			d += 2 * (x - y) + 1;
		}
		
		/*画每个八分之一圆弧的点*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X + y, Y + x);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - y, Y - x);
		OLED_DrawPoint(X + x, Y - y);
		OLED_DrawPoint(X + y, Y - x);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X - y, Y + x);
		
		if (IsFilled)	//指定圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分圆*/
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
			
			/*遍历两侧部分*/
			for (j = -x; j < x; j ++)
			{
				/*在指定区域画点，填充部分圆*/
				OLED_DrawPoint(X - y, Y + j);
				OLED_DrawPoint(X + y, Y + j);
			}
		}
	}
}


/**
 * @brief 画椭圆
 * @param X 圆心的横坐标
 * @param Y 圆心的纵坐标
 * @param A 椭圆的横向半轴长度	
 * @param B 椭圆的纵向半轴长度
 * @param IsFilled 是否填充
 * @return 无
 */
void OLED_DrawEllipse(int16_t X, int16_t Y, int16_t A, int16_t B, uint8_t IsFilled)
{
	int16_t x, y, j;
	int16_t a = A, b = B;
	float d1, d2;
	
	/*使用Bresenham算法画椭圆，可以避免部分耗时的浮点运算，效率更高*/
	/*参考链接：https://blog.csdn.net/myf_666/article/details/128167392*/
	
	x = 0;
	y = b;
	d1 = b * b + a * a * (-b + 0.5);
	
	if (IsFilled)	//指定椭圆填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在指定区域画点，填充部分椭圆*/
			OLED_DrawPoint(X, Y + j);
			OLED_DrawPoint(X, Y + j);
		}
	}
	
	/*画椭圆弧的起始点*/
	OLED_DrawPoint(X + x, Y + y);
	OLED_DrawPoint(X - x, Y - y);
	OLED_DrawPoint(X - x, Y + y);
	OLED_DrawPoint(X + x, Y - y);
	
	/*画椭圆中间部分*/
	while (b * b * (x + 1) < a * a * (y - 0.5))
	{
		if (d1 <= 0)		//下一个点在当前点东方
		{
			d1 += b * b * (2 * x + 3);
		}
		else				//下一个点在当前点东南方
		{
			d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2);
			y --;
		}
		x ++;
		
		if (IsFilled)	//指定椭圆填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分椭圆*/
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}
		
		/*画椭圆中间部分圆弧*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}
	
	/*画椭圆两侧部分*/
	d2 = b * b * (x + 0.5) * (x + 0.5) + a * a * (y - 1) * (y - 1) - a * a * b * b;
	
	while (y > 0)
	{
		if (d2 <= 0)		//下一个点在当前点东方
		{
			d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3);
			x ++;
			
		}
		else				//下一个点在当前点东南方
		{
			d2 += a * a * (-2 * y + 3);
		}
		y --;
		
		if (IsFilled)	//指定椭圆填充
		{
			/*遍历两侧部分*/
			for (j = -y; j < y; j ++)
			{
				/*在指定区域画点，填充部分椭圆*/
				OLED_DrawPoint(X + x, Y + j);
				OLED_DrawPoint(X - x, Y + j);
			}
		}
		
		/*画椭圆两侧部分圆弧*/
		OLED_DrawPoint(X + x, Y + y);
		OLED_DrawPoint(X - x, Y - y);
		OLED_DrawPoint(X - x, Y + y);
		OLED_DrawPoint(X + x, Y - y);
	}
}


/**
 * @brief 画圆弧
 * @param X 圆心的横坐标
 * @param Y 圆心的纵坐标
 * @param Radius 圆弧的半径
 * @param StartAngle 圆弧的起始角度
 * @param EndAngle 圆弧的终止角度
 * @param IsFilled 是否填充
 * @return 无
 */
void OLED_DrawArc(int16_t X, int16_t Y, int16_t Radius, int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
	int16_t x, y, d, j;
	if(Radius <=0){return;} //半径为0，直接返回
	/*此函数借用Bresenham算法画圆的方法*/
	
	d = 1 - Radius;
	x = 0;
	y = Radius;
	
	/*在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
	if (OLED_IsInAngle(x, y, StartAngle, EndAngle))	{OLED_DrawPoint(X + x, Y + y);}
	if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y - y);}
	if (OLED_IsInAngle(y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + x);}
	if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y - x);}
	
	if (IsFilled)	//指定圆弧填充
	{
		/*遍历起始点Y坐标*/
		for (j = -y; j < y; j ++)
		{
			/*在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
			if (OLED_IsInAngle(0, j, StartAngle, EndAngle)) {OLED_DrawPoint(X, Y + j);}
		}
	}
	
	while (x < y)		//遍历X轴的每个点
	{
		x ++;
		if (d < 0)		//下一个点在当前点东方
		{
			d += 2 * x + 1;
		}
		else			//下一个点在当前点东南方
		{
			y --;
			d += 2 * (x - y) + 1;
		}
		
		/*在画圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
		if (OLED_IsInAngle(x, y, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y + y);}
		if (OLED_IsInAngle(y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + x);}
		if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y - y);}
		if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y - x);}
		if (OLED_IsInAngle(x, -y, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y - y);}
		if (OLED_IsInAngle(y, -x, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y - x);}
		if (OLED_IsInAngle(-x, y, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y + y);}
		if (OLED_IsInAngle(-y, x, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y + x);}
		
		if (IsFilled)	//指定圆弧填充
		{
			/*遍历中间部分*/
			for (j = -y; j < y; j ++)
			{
				/*在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
				if (OLED_IsInAngle(x, j, StartAngle, EndAngle)) {OLED_DrawPoint(X + x, Y + j);}
				if (OLED_IsInAngle(-x, j, StartAngle, EndAngle)) {OLED_DrawPoint(X - x, Y + j);}
			}
			
			/*遍历两侧部分*/
			for (j = -x; j < x; j ++)
			{
				/*在填充圆的每个点时，判断指定点是否在指定角度内，在，则画点，不在，则不做处理*/
				if (OLED_IsInAngle(-y, j, StartAngle, EndAngle)) {OLED_DrawPoint(X - y, Y + j);}
				if (OLED_IsInAngle(y, j, StartAngle, EndAngle)) {OLED_DrawPoint(X + y, Y + j);}
			}
		}
	}
}



/**
 * @brief 画圆角矩形
 * @param X 矩形左上角的横坐标
 * @param Y 矩形左上角的纵坐标
 * @param Width 矩形的宽度
 * @param Height 矩形的高度
 * @param Radius 圆角半径
 * @param IsFilled 是否填充
 * @return 无
 */
void OLED_DrawRoundedRectangle(int16_t X, int16_t Y, int16_t Width, int16_t Height, int16_t Radius, uint8_t IsFilled)
{
    // 安全检查
    if (Width == 0 || Height == 0) return;  // 宽度或高度为0,直接返回
    if (Radius > Width / 2 || Radius > Height / 2) {
        Radius = (Width < Height ? Width : Height) / 2;  // 限制圆角半径不超过宽高的一半
    }
    if (Radius <= 0) {
        OLED_DrawRectangle(X, Y, Width, Height, IsFilled);  // 如果半径为0,退化为普通矩形
        return;
    }
		// 绘制四个圆角
		OLED_DrawArc(X + Radius, Y + Radius, Radius,180,  -90, IsFilled);
		OLED_DrawArc(X + Width - Radius - 1, Y + Radius, Radius,-90,  0, IsFilled);
		OLED_DrawArc(X + Radius, Y + Height - Radius - 1, Radius,90,  180, IsFilled);
		OLED_DrawArc(X + Width - Radius - 1, Y + Height - Radius - 1, Radius,0,  90, IsFilled);


    // 填充或绘制矩形主体
    if (IsFilled) {
        OLED_DrawRectangle(X + Radius, Y, Width - 2 * Radius, Height, OLED_FILLED);
        OLED_DrawRectangle(X, Y + Radius, Width, Height - 2 * Radius, OLED_FILLED);
    } else {
        // 绘制顶部和底部的直线
        OLED_DrawLine(X + Radius, Y, X + Width - Radius - 1, Y);
        OLED_DrawLine(X + Radius, Y + Height - 1, X + Width - Radius - 1, Y + Height - 1);
        // 绘制左侧和右侧的直线
        OLED_DrawLine(X, Y + Radius, X, Y + Height - Radius - 1);
        OLED_DrawLine(X + Width - 1, Y + Radius, X + Width - 1, Y + Height - Radius - 1);
    }
}







