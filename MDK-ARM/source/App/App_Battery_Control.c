#include "App_Battery_Control.h"
#include "Int_SC8815.h"
#include "Int_BQ76952.h"
#include <stdio.h>

typedef enum
{
    App_Battery_Control_Reason_NONE,
    App_Battery_Control_Reason_ERROR,
    App_Battery_Control_Reason_ERROR_SC8815_COMM,
    App_Battery_Control_Reason_ERROR_SC8815_SAFE,
    App_Battery_Control_Reason_ERROR_BQ76952_COMM,
    App_Battery_Control_Reason_ERROR_BQ76952_SAFE,
    App_Battery_Control_Reason_ERROR_BQ76952_SAMPLE_SAFE
} App_Battery_Control_Reason;

typedef enum
{
    App_Battery_Control_State_Idle,
    App_Battery_Control_State_Prepare,
    App_Battery_Control_State_Start,
    App_Battery_Control_State_Running,
    App_Battery_Control_State_End
} App_Battery_Control_StateDef;

typedef struct
{
    bool do_work;
    bool charge_request;
    bool charge_done;
    uint8_t ac_ok_count;
    uint8_t ac_off_count;
    uint32_t charge_end_tick;
    App_Battery_Control_Reason reason;
    App_Battery_Control_StateDef state;
} App_Battery_Control_Data;

App_Battery_Control_Data control_data;

/**
 * BMS项目 - 电池控制
 */
void App_Battery_Control_Init(void)
{
    control_data.state = App_Battery_Control_State_Idle;
    control_data.charge_request = false;
    control_data.ac_ok_count = 0;
    control_data.charge_done = true;
    control_data.do_work = false;
}

void __App_Battery_Control_Flow(App_Battery_Control_StateDef state, App_Battery_Control_Reason reason)
{
    printf("[Charge] Contro Flow : %d, %d \n", state, reason);
    control_data.state = state;
    control_data.reason = reason;
}

/**
 * BMS项目 - 轮询
 */
void App_Battery_Control_Poll(void)
{

    switch (control_data.state)
    {
    case App_Battery_Control_State_Idle:
    {

        if (control_data.charge_done == false)
        {
            break;
        }

        // !获取SC8815状态
        Int_SC8815_StatusFlagsTypeDef sc8815_reg_status;
        Int_SC8815_StatusTypeDef sc8815_status = Int_SC8815_ReadStatus(&sc8815_reg_status);
        if (sc8815_status != INT_SC8815_OK)
        {
            break;
        }

        // DC电源适配器在接入时，会抖动
        if ( control_data.ac_ok_count == 99 ) {
            // 新的逻辑
            if (sc8815_reg_status.ac_ok == false) {
                control_data.ac_off_count++;
            } else {
                control_data.ac_off_count = 0;
            }
            if ( control_data.ac_off_count >= 3 ) {
                control_data.ac_ok_count = 0;
            }
            break;
        } else {
            if (sc8815_reg_status.ac_ok == true)
            {
                control_data.ac_ok_count++;
            }
            else
            {
                control_data.ac_ok_count = 0;
            }
        }

        if (control_data.ac_ok_count < 3)
        {
            break;
        }
        // 电源已经接入
        if (sc8815_reg_status.otp == true || sc8815_reg_status.vbus_short == true)
        {
            break;
        }
        // !获取BQ76952状态
        App_Sample_Data sample_data;
        bool ret = App_Sample_Data_Receive(&sample_data);
        if (ret == false)
        {
            break;
        }

        if (sample_data.safety_valid == false)
        {
            break;
        }
        control_data.charge_request = true;
        __App_Battery_Control_Flow(App_Battery_Control_State_Prepare, App_Battery_Control_Reason_NONE);
        break;
    }
    case App_Battery_Control_State_Prepare:
    {
        if (control_data.do_work == false)
        {
            control_data.do_work = true;

            // !判断状态
            Int_SC8815_StatusFlagsTypeDef sc8815_reg_status;
            Int_SC8815_StatusTypeDef sc8815_status = Int_SC8815_ReadStatus(&sc8815_reg_status);
            if (sc8815_status != INT_SC8815_OK)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_COMM);
                break;
            }

            if (sc8815_reg_status.ac_ok == false || sc8815_reg_status.otp == true || sc8815_reg_status.vbus_short == true)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_SAFE);
                break;
            }
            // !下发配置
            sc8815_status = Int_SC8815_Update_Config();
            if (sc8815_status != INT_SC8815_OK)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_COMM);
                break;
            }

            __App_Battery_Control_Flow(App_Battery_Control_State_Start, App_Battery_Control_Reason_NONE);

            control_data.do_work = false;
        }

        break;
    }
    case App_Battery_Control_State_Start:
    {
        if (control_data.do_work == false)
        {
            control_data.do_work = true;

            // !判断状态
            Int_SC8815_StatusFlagsTypeDef sc8815_reg_status;
            Int_SC8815_StatusTypeDef sc8815_status = Int_SC8815_ReadStatus(&sc8815_reg_status);
            if (sc8815_status != INT_SC8815_OK)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_COMM);
                break;
            }

            if (sc8815_reg_status.ac_ok == false || sc8815_reg_status.otp == true || sc8815_reg_status.vbus_short == true)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_SAFE);
                break;
            }
            // !打开FET开关（BQ, SC）
            App_BQ_Bus_Lock();
            BQ76952_StatusDef bq_status = Int_BQ76952_OP_FET_Set_Mode(BQ76952_FET_MODE_ALL_ON);
            App_BQ_Bus_unLock();

            if (bq_status != BQ76952_Status_OK)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_BQ76952_COMM);
                break;
            }

            sc8815_status = Int_SC8815_Start_Charge();
            if (sc8815_status != INT_SC8815_OK)
            {
                __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_COMM);
                break;
            }
            control_data.charge_end_tick = HAL_GetTick() + 1000 * 10 * 1;
            __App_Battery_Control_Flow(App_Battery_Control_State_Running, App_Battery_Control_Reason_NONE);

            control_data.do_work = false;
        }

        break;
    }
    case App_Battery_Control_State_Running:
    {
        // !判断状态
        Int_SC8815_StatusFlagsTypeDef sc8815_reg_status;
        Int_SC8815_StatusTypeDef sc8815_status = Int_SC8815_ReadStatus(&sc8815_reg_status);
        if (sc8815_status != INT_SC8815_OK)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_COMM);
            break;
        }

        if (sc8815_reg_status.ac_ok == false || sc8815_reg_status.otp == true || sc8815_reg_status.vbus_short == true)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_SC8815_SAFE);
            break;
        }

        // !获取BQ76952状态
        App_Sample_Data sample_data;
        bool ret = App_Sample_Data_Receive(&sample_data);
        if (ret == false)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_BQ76952_SAFE);
            break;
        }

        if (sample_data.safety_valid == false)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_BQ76952_SAFE);
            break;
        }

        // 采集数据太旧不行的
        uint32_t now = HAL_GetTick();
        if ((now - sample_data.sample_tick) > 1000 * 50)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_ERROR_BQ76952_SAMPLE_SAFE);
            break;
        }

        // !时间判断
        if (now >= control_data.charge_end_tick)
        {
            __App_Battery_Control_Flow(App_Battery_Control_State_End, App_Battery_Control_Reason_NONE);
            break;
        }
        break;
    }
    case App_Battery_Control_State_End:
    {
        // !关闭FET
        Int_SC8815_StatusTypeDef sc8815_status = Int_SC8815_Stop_Charge();
        App_BQ_Bus_Lock();
        BQ76952_StatusDef bq_status = Int_BQ76952_OP_FET_Set_Mode(BQ76952_FET_MODE_ALL_OFF);
        App_BQ_Bus_unLock();

        if (control_data.reason != App_Battery_Control_Reason_NONE)
        {
        }
        control_data.do_work = false;
        control_data.ac_ok_count = 99;
        control_data.charge_done = true;
        control_data.charge_end_tick = 0;
        control_data.reason = App_Battery_Control_Reason_NONE;
        control_data.charge_request = false;
        control_data.state = App_Battery_Control_State_Idle;
        break;
    }
    default:
        break;
    }
}