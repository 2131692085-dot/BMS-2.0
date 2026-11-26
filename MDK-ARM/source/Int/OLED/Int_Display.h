#ifndef __INT_DISPLAY_H__
#define __INT_DISPLAY_H__

#include "stm32g0xx_hal.h"

/**
 * 屏幕显示 - 初始化
 */
void Int_Display_Init(void);

/**
 * 屏幕显示 - 渲染
 */
void Int_Display_Render(uint8_t init) ;

#endif /* __INT_DISPLAY_H__ */
