#include "App_Main.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Int_Display.h"
#include "App_Battery_Manager.h"
#include "App_Battery_Control.h"
#include "queue.h"
#include "semphr.h"
#include "Int_CAN.h"


enum {
    TASK_PRIORITY_0,
    TASK_PRIORITY_1,
    TASK_PRIORITY_2,
    TASK_PRIORITY_3,
    TASK_PRIORITY_4,
} ;

/**
 * BMS - 电池管理任务
 */
void __battery_manager_task(void *parameter);
/**
 * BMS - 电池控制任务
 */
void __battery_control_task(void *parameter);
/**
 * BMS - 电池远程控制任务
 */
void __battery_remote_task(void *parameter);

SemaphoreHandle_t bq_iic_semaphore;
QueueHandle_t bq_sample_queue;


/**
 * BMS项目 - 传输采样数据
 */
bool App_Sample_Data_Send(App_Sample_Data sample_data) {
    return ( xQueueOverwrite(bq_sample_queue, &sample_data) == pdPASS );
}
/**
 * BMS项目 - 接收采样数据
 */
bool App_Sample_Data_Receive(App_Sample_Data *sample_data) {
    return (xQueuePeek(bq_sample_queue, sample_data, 0) == pdTRUE );
}

/**
 * BMS项目 - BQ IIC总线锁定
 */
void App_BQ_Bus_Lock(void) {
    xSemaphoreTake(bq_iic_semaphore, portMAX_DELAY);
}
/**
 * BMS项目 - BQ IIC总线释放锁定
 */
void App_BQ_Bus_unLock(void) {
    xSemaphoreGive(bq_iic_semaphore);
}

/**
 * BMS项目 - 初始化
 */
void __App_Main_Init() {
    App_Battery_Control_Init();
    App_Battery_Manager_Init();

    bq_iic_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(bq_iic_semaphore);

    bq_sample_queue = xQueueCreate(1, sizeof(App_Sample_Data));
}

/**
 * BMS项目 - 入口函数
 */
void App_main(void) {
    printf("BMS Project Start \n");

    //__App_Main_Init();
    Int_CAN_Init();

    // 创建任务
    //xTaskCreate(__battery_manager_task, "__battery_manager_task", 128 * 6, NULL, TASK_PRIORITY_4, NULL);
    //xTaskCreate(__battery_control_task, "__battery_control_task", 128 * 4, NULL, TASK_PRIORITY_4, NULL);
    xTaskCreate(__battery_remote_task, "__battery_remote_task", 128 * 4, NULL, TASK_PRIORITY_4, NULL);
    vTaskStartScheduler();
}

/**
 * BMS - 电池远程控制任务
 */
void __battery_remote_task(void *parameter) {
    while (1) {
        Int_CAN_Send_Data("bms", 3);
        printf("FD Can Send Data !!!\n");
        vTaskDelay(2000);
    }
}

/**
 * BMS - 电池管理任务
 */
void __battery_manager_task(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_time = 0;
    uint32_t oled_refresh_cnt = 0;
    while (1) {
        App_Sample_Data sample_Data;
        uint32_t now = HAL_GetTick();
        if ( last_time == 0 ) {
            last_time = now;
        }
        uint32_t dt = now - last_time;
        App_BQ_Bus_Lock();
        App_Battery_Manager_Poll(dt, &sample_Data);
        App_BQ_Bus_unLock();

        // 数据采集时间
        sample_Data.sample_tick = now;
        App_Sample_Data_Send(sample_Data);
        last_time = now;

        // 刷新OLED
        oled_refresh_cnt++;
        if ( oled_refresh_cnt == 60 ) {
            App_Battery_Manager_Refresh_OLED();
            oled_refresh_cnt = 0;
        }
        vTaskDelayUntil( &xLastWakeTime, 10000 );
    }
}
/**
 * BMS - 电池控制任务
 */
void __battery_control_task(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();;
    while (1) {
        App_Battery_Control_Poll();
        vTaskDelayUntil( &xLastWakeTime, 100 );
    }
}
