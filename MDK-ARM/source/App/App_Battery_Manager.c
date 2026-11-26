#include "App_Battery_Manager.h"
#include "Int_Display.h"
#include "Int_BQ76952.h"
#include "Int_SC8815.h"
#include <stdio.h>

/**
 * BMS - 电池管理 - 初始化
 */
void App_Battery_Manager_Init(void)
{
    Int_Display_Init();
    Int_BQ76952_Init();

    // !将BQ76952芯片进行复位
    uint8_t try_cnt = 10;
    while (try_cnt)
    {
        printf("BQ76952 Reset Ing!!! \n");
        BQ76952_StatusDef status = Int_BQ76952_OP_Reset();
        if (status == BQ76952_Status_OK)
        {
            break;
        }
        try_cnt--;
        HAL_Delay(1000);
    }

    if (try_cnt == 0)
    {
        printf("BQ76952 Reset Fail \n");
        return;
    }
    printf("BQ76952 Reset Success!!! \n");
    // 芯片要求 ，复位重启后应该间隔一段时间(300ms)才能执行其他操作
    HAL_Delay(300);

    // !禁用BQ76952芯片睡眠
    BQ76952_StatusDef status = Int_BQ76952_OP_Disable_Sleep_Mode();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Disable_Sleep_Mode Fail \n");
        return;
    }
    printf("BQ76952 Disable_Sleep_Mode Success!!! \n");

    // !获取设备编号
    status = Int_BQ76952_OP_Get_Device_Number();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Get_Device_Number Fail \n");
        return;
    }

    // !修改配置
    status = Int_BQ76952_OP_Config_Update();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Config Update Fail \n");
        return;
    }
    // !清除启动告警
    status = Int_BQ76952_OP_Clear_Startup_Alarm();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Clear Startup Alarm Fail \n");
        return;
    }
    // !关闭MOS管开关
    status = Int_BQ76952_OP_Close_All_FET();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Close All FET Fail : %d \n", status);
        return;
    }
    // !数据采样
    status = Int_BQ76952_OP_Sample();
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Sample Fail \n");
        return;
    }

    // !更新电阻RC模型
    status = Int_BQ76952_OP_Update_RC_Model(0);
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Update_RC_Model Fail \n");
        return;
    }

    // !更新电池容量百分比
    status = Int_BQ76952_OP_Update_SOC(0);
    if (status != BQ76952_Status_OK)
    {
        printf("BQ76952 Update_SOC Fail \n");
        return;
    }

    Int_Display_Render(1);

    Int_SC8815_Init();

    printf("App Battery Manager Init Success!!! \n");
}

/**
 * BMS - 电池管理 - 轮询
 */
void App_Battery_Manager_Poll(uint32_t dt, App_Sample_Data *sample_Data) {
    
    BQ76952_StatusDef status = Int_BQ76952_OP_Sample();
    if (status != BQ76952_Status_OK)
    {
        return;
    }
    // !更新电阻RC模型
    status = Int_BQ76952_OP_Update_RC_Model(dt);
    if (status != BQ76952_Status_OK)
    {
        return;
    }

    // !更新电池容量百分比
    status = Int_BQ76952_OP_Update_SOC(dt);
    if (status != BQ76952_Status_OK)
    {
        return;
    }

    // ! 均衡处理
    Int_BQ76952_OP_Cell_Balance();

    BQ76952_Sample bq_sample;
    Int_BQ76952_OP_Get_Sample_Data(&bq_sample);

    sample_Data->safety_valid = (bq_sample.alarm_status.global_safety_fault == 0);
    sample_Data->sample_valid = (bq_sample.alarm_status.global_fault == 0);
    sample_Data->soc = bq_sample.soc.display_percent;
}

/**
 * BMS - 电池管理 - 刷新SOC的显示
 */
void App_Battery_Manager_Refresh_OLED(void) {
    Int_Display_Render(0);
}
