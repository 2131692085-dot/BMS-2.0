#ifndef __APP_BATTERY_CONTROL_H__
#define __APP_BATTERY_CONTROL_H__

#include "stm32g0xx_hal.h"
#include "App_Main.h"

/**
 * BMS项目 - 电池控制
 */
void App_Battery_Control_Init(void);
/**
 * BMS项目 - 轮询
 */
void App_Battery_Control_Poll(void);

#endif /* __APP_BATTERY_CONTROL_H__ */
