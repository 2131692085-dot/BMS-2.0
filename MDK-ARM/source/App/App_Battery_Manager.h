#ifndef __APP_BATTERY_MANAGER_H__
#define __APP_BATTERY_MANAGER_H__

#include "stm32g0xx_hal.h"
#include "App_Main.h"

/**
 * BMS - 电池管理 - 初始化
 */
void App_Battery_Manager_Init(void);
/**
 * BMS - 电池管理 - 轮询
 */
void App_Battery_Manager_Poll(uint32_t dt, App_Sample_Data *sample_Data);

#endif /* __APP_BATTERY_MANAGER_H__ */
