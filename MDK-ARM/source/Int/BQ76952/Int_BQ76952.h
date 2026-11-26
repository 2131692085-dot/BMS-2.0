#ifndef __INT_BQ76952_H__
#define __INT_BQ76952_H__

#include "stm32g0xx_hal.h"

typedef enum {
    BQ76952_Status_OK,
    BQ76952_Status_ERR,
    BQ76952_Status_ERR_Send_Direct_CMD_CRC,
    BQ76952_Status_ERR_Send_Direct_CMD_NO_CRC,
    BQ76952_Status_ERR_Send_Sub_CMD,
    BQ76952_Status_ERR_Send_Sub_CMD_ECHO,
    BQ76952_Status_ERR_Send_Sub_CMD_CHECKSUM,
    BQ76952_Status_ERR_FET_Fail,
    BQ76952_Status_ERR_FET_SW_Fail,
} BQ76952_StatusDef;

typedef enum {
    BQ76952_CRC_Status_YES,
    BQ76952_CRC_Status_NO
} BQ76952_CRC_StatusDef;

typedef enum {
    BQ76952_Debug_Status_YES,
    BQ76952_Debug_Status_NO
} BQ76952_Debug_StatusDef;

typedef enum
{
    BQ76952_FET_MODE_CHG,
    BQ76952_FET_MODE_DSG,
    BQ76952_FET_MODE_ALL_ON,
    BQ76952_FET_MODE_ALL_OFF
} BQ76952_FET_Mode;

typedef struct
{
    BQ76952_CRC_StatusDef crc_status;
    BQ76952_Debug_StatusDef debug_status;
    uint32_t last_balance_tick;
    uint16_t last_balance_mask;
    uint8_t last_balance_index;
} BQ76952_Attribute;
// ***********************************************************
typedef struct
{
    uint8_t sample_valid;
    uint16_t vcell[6];
    uint16_t cell_max_mv;
    uint16_t cell_max_index;
    uint16_t cell_min_mv;
    uint16_t cell_sum_mv;
    uint16_t cell_avg_mv;
    uint16_t cell_delta_mv;
    uint8_t cell_valid_count;
    uint16_t cell_stack_mv;
} BQ76952_Sample_Voltage;

typedef struct
{
    uint8_t sample_valid;
    int16_t current_ma;
    float current_a;
} BQ76952_Sample_Current;

typedef struct
{
    int16_t ic;
    int16_t ts1;
    int16_t ts3;
    int16_t cell;
    int16_t fet;
    uint8_t sample_ic_valid;
    uint8_t sample_ts1_valid;
    uint8_t sample_ts3_valid;
    uint8_t sample_cell_valid;
} BQ76952_Sample_Temperature;

typedef struct
{
    uint16_t alarm_status;
    uint16_t alarm_raw;
    uint16_t battery_status;
    uint16_t manufacturing_status;
    uint8_t fet_status;
    uint8_t fet_control_request;
    uint8_t safety_alert_a;
    uint8_t safety_alert_b;
    uint8_t safety_alert_c;
    uint8_t safety_status_a;
    uint8_t safety_status_b;
    uint8_t safety_status_c;
    uint8_t pf_status_a;
    uint8_t pf_status_b;
    uint8_t pf_status_c;
    uint8_t pf_status_d;
    uint8_t sample_valid;
    uint8_t global_fault;
    uint8_t global_cell_fault;
    uint8_t global_safety_fault;
} BQ76952_Sample_Alarm_Status;

typedef struct
{
    uint16_t cell_min_mv;
    uint16_t cell_avg_mv;
    int16_t cell_ohmic_mv;
    int16_t cell_polar_mv;
    int16_t cell_total_mv;
    int32_t polar_mv; // 全局极化内阻
} BQ76952_RC_Model;

typedef struct
{
    float percent;
    float display_percent;
    uint8_t confidence_percent;
    float residual_percent;
    float kalman_gain;
    float p;
    float active_capacity_mah;
    uint8_t full_anchor_used;
    uint8_t empty_anchor_used;
} BQ76952_SOC;

typedef struct
{
    BQ76952_Sample_Voltage voltage;
    BQ76952_Sample_Current current;
    BQ76952_Sample_Temperature temperature;
    BQ76952_Sample_Alarm_Status alarm_status;
    uint8_t sample_valid;
    BQ76952_RC_Model rc_model;
    BQ76952_SOC soc;
} BQ76952_Sample;

/**
 * BQ76952管理芯片 - 初始化
 */
void Int_BQ76952_Init(void);

/**
 * BQ76952管理芯片 - 复位
 */
BQ76952_StatusDef Int_BQ76952_OP_Reset(void);
/**
 * BQ76952管理芯片 - 禁用睡眠
 */
BQ76952_StatusDef Int_BQ76952_OP_Disable_Sleep_Mode(void);
/**
 * BQ76952管理芯片 - 获取设备编号
 */
BQ76952_StatusDef Int_BQ76952_OP_Get_Device_Number(void);
/**
 * BQ76952管理芯片 - 修改配置
 */
BQ76952_StatusDef Int_BQ76952_OP_Config_Update(void);
/**
 * BQ76952管理芯片 - 清除启动告警
 */
BQ76952_StatusDef Int_BQ76952_OP_Clear_Startup_Alarm(void);
/**
 * BQ76952管理芯片 - 关闭MOS管开关
 */
BQ76952_StatusDef Int_BQ76952_OP_Close_All_FET(void);
/**
 * BQ76952管理芯片 - 数据采样
 */
BQ76952_StatusDef Int_BQ76952_OP_Sample(void);
/**
 * BQ76952管理芯片 - 设置FET模式
 */
BQ76952_StatusDef Int_BQ76952_OP_FET_Set_Mode(BQ76952_FET_Mode fet_mode);

/**
 * BQ76952管理芯片 - 更新RC电阻模型
 */
BQ76952_StatusDef Int_BQ76952_OP_Update_RC_Model(uint16_t interval_ms);

/**
 * BQ76952管理芯片 - 更新电池容量百分比
 */
BQ76952_StatusDef Int_BQ76952_OP_Update_SOC(uint32_t interval_ms);

/**
 * BQ76952管理芯片 - 获取SOC
 */
BQ76952_StatusDef Int_BQ76952_OP_Get_SOC(uint8_t init, uint8_t *soc);

void Int_BQ76952_OP_Get_Sample_Data(BQ76952_Sample *bq_sample);
/**
 * BQ76952管理芯片 - 被动均衡（外部）
 */
BQ76952_StatusDef Int_BQ76952_OP_Cell_Balance(void);

#endif /* __INT_BQ76952_H__ */
