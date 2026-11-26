#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#include "stm32g0xx_hal.h"
#include <stdbool.h>

typedef struct
{
    uint32_t sample_tick;
    uint8_t sample_valid;
    uint8_t safety_valid;
    uint8_t soc;
} App_Sample_Data;

/**
 * BMS项目 - 入口函数
 */
void App_main(void);

/**
 * BMS项目 - BQ IIC总线锁定
 */
void App_BQ_Bus_Lock(void);
/**
 * BMS项目 - BQ IIC总线释放锁定
 */
void App_BQ_Bus_unLock(void);
/**
 * BMS项目 - 传输采样数据
 */
bool App_Sample_Data_Send(App_Sample_Data sample_data);
/**
 * BMS项目 - 接收采样数据
 */
bool App_Sample_Data_Receive(App_Sample_Data *sample_data);

#endif /* __APP_MAIN_H__ */
