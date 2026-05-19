#ifndef __OLED_DRIVER_H
#define __OLED_DRIVER_H

#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <ch32v00x.h> // port to ch32v00x

#ifdef __cplusplus
extern "C" {
#endif

//	oled初始化函数
void OLED_Init(void);
//	oled全局刷新函数
void OLED_Update(void);
//	oled局部刷新函数
void OLED_UpdateArea(int16_t X, int16_t Y, int16_t Width, int16_t Height);
// 设置颜色模式
void OLED_SetColorMode(bool colormode);
// OLED 设置亮度函数
void OLED_SetBrightness(int16_t Brightness);
// 显存数组是否变化
bool OLED_IfChangedScreen(void);

#ifdef __cplusplus
}
#endif


#endif







