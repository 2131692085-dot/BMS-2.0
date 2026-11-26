#include "Int_BQ76952.h"
#include "i2c.h"
#include "Int_BQ76952_BSP.h"
#include <stdio.h>
#include <stdbool.h>
#include "Com_SOC.h"
#include "FreeRTOS.h"
#include "task.h"

BQ76952_Attribute s_bq76952_attribute = {
    .crc_status = BQ76952_CRC_Status_NO,
    .debug_status = BQ76952_Debug_Status_YES
};

BQ76952_Sample s_sample;

enum
{
    /*
     * 项目 Data Memory 基线值。
     */
    APP_BATMAN_DM_DA_CONFIGURATION_DEFAULT = 0x05u,
    APP_BATMAN_DM_PROTECTION_CONFIGURATION_DEFAULT = 0x0002u,
    APP_BATMAN_DM_ENABLED_PROTECTIONS_A_DEFAULT = BQ76952_ENABLED_PROTECTIONS_A_SCD_MASK |
        BQ76952_ENABLED_PROTECTIONS_A_OCD2_MASK |
        BQ76952_ENABLED_PROTECTIONS_A_OCD1_MASK |
        BQ76952_ENABLED_PROTECTIONS_A_OCC_MASK |
        BQ76952_ENABLED_PROTECTIONS_A_CUV_MASK |
        BQ76952_ENABLED_PROTECTIONS_A_COV_MASK,
    APP_BATMAN_DM_ENABLED_PROTECTIONS_B_DEFAULT = BQ76952_ENABLED_PROTECTIONS_B_OTD_MASK |
        BQ76952_ENABLED_PROTECTIONS_B_OTC_MASK,
    APP_BATMAN_DM_ENABLED_PROTECTIONS_C_DEFAULT = 0x00u,
    APP_BATMAN_DM_CHG_FET_PROTECTIONS_A_DEFAULT = 0x98u,
    APP_BATMAN_DM_CHG_FET_PROTECTIONS_B_DEFAULT = 0xD5u,
    APP_BATMAN_DM_CHG_FET_PROTECTIONS_C_DEFAULT = 0x56u,
    APP_BATMAN_DM_DSG_FET_PROTECTIONS_A_DEFAULT = 0xE4u,
    APP_BATMAN_DM_DSG_FET_PROTECTIONS_B_DEFAULT = 0xE6u,
    APP_BATMAN_DM_DSG_FET_PROTECTIONS_C_DEFAULT = 0xE2u,
    APP_BATMAN_DM_DEFAULT_ALARM_MASK_DEFAULT = 0xF800u,

    /* 5mΩ 实板采样电阻对应的 IEEE-754 小端写入值。 */
    APP_BATMAN_DM_CC_GAIN_5_MOHM_IEEE754 = 0x3FBF67F5u,
    APP_BATMAN_DM_CAPACITY_GAIN_5_MOHM_IEEE754 = 0x48D9C710u,

    APP_BATMAN_DM_CUV_THRESHOLD_2V83 = 56u,    /* 约 2.83V。 */
    APP_BATMAN_DM_CUV_DELAY_1S = 300u,         /* 约 996.6ms。 */
    APP_BATMAN_DM_CUV_RECOVERY_HYS_200MV = 4u, /* 约 202mV。 */
    APP_BATMAN_DM_OCC_THRESHOLD_6A = 15u,
    APP_BATMAN_DM_OCC_DELAY_426MS = 127u,
    APP_BATMAN_DM_OCD1_THRESHOLD_14A = 35u,
    APP_BATMAN_DM_OCD1_DELAY_300MS = 89u,
    APP_BATMAN_DM_OCD2_THRESHOLD_15A2 = 38u,
    APP_BATMAN_DM_OCD2_DELAY_80MS = 22u,
    APP_BATMAN_DM_SCD_THRESHOLD_80MV = 0x04u, /* 5mΩ 下约 16A。 */
    APP_BATMAN_DM_SCD_DELAY_400US = 0x1Cu,
    APP_BATMAN_DM_OCC_RECOVERY_NEG_200MA = 0xFF38u,
    APP_BATMAN_DM_OCD_RECOVERY_500MA = 500u,
    APP_BATMAN_DM_OTC_THRESHOLD_50C = 50u,
    APP_BATMAN_DM_OTC_DELAY_3S = 3u,
    APP_BATMAN_DM_OTC_RECOVERY_45C = 45u,
    APP_BATMAN_DM_OTD_THRESHOLD_60C = 60u,
    APP_BATMAN_DM_OTD_DELAY_3S = 3u,
    APP_BATMAN_DM_OTD_RECOVERY_55C = 55u,

    APP_BATMAN_DM_FET_OPTIONS_DEFAULT = BQ76952_FET_OPTIONS_FET_INIT_OFF_MASK |
        BQ76952_FET_OPTIONS_PDSG_EN_MASK |
        BQ76952_FET_OPTIONS_FET_CTRL_EN_MASK |
        BQ76952_FET_OPTIONS_HOST_FET_EN_MASK |
        BQ76952_FET_OPTIONS_SFET_MASK,
    APP_BATMAN_DM_CHG_PUMP_CONTROL_DEFAULT = 0x01u,
    APP_BATMAN_DM_PREDISCHARGE_TIMEOUT_2500MS = 250u,
    APP_BATMAN_DM_PREDISCHARGE_STOP_DELTA_DISABLED = 0u,
    APP_BATMAN_DM_BALANCING_CONFIGURATION_DEFAULT = 0x00u,
    APP_BATMAN_MAIN_FET_OFF_MASK = BQ76952_FET_CONTROL_PCHG_OFF_MASK |
        BQ76952_FET_CONTROL_CHG_OFF_MASK |
        BQ76952_FET_CONTROL_PDSG_OFF_MASK |
        BQ76952_FET_CONTROL_DSG_OFF_MASK
};

/**
 * @brief 将一个字节并入 BQ76952 CRC-8 累计值。
 *
 * 算法按 MSB-first 处理，使用 BSP 中定义的 0x07 多项式；不做最终异或。
 *
 * @param crc 前一字节处理后的 CRC 值。
 * @param data 本次并入的协议字节。
 * @return 更新后的 CRC 值。
 */
static uint8_t __Int_BQ76952_Crc8Update(uint8_t crc, uint8_t data)
{
    crc ^= data;

    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
        if ((crc & 0x80u) != 0u)
        {
            crc = (uint8_t)((crc << 1u) ^ BQ76952_CRC8_POLY);
        }
        else
        {
            crc = (uint8_t)(crc << 1u);
        }
    }

    return crc;
}

/**
 * @brief 从 BQ76952 规定的初值开始计算一段数据的 CRC-8。
 *
 * @param data 参与 CRC 的连续字节。
 * @param len 字节数。
 * @return CRC-8 校验值。
 */
static uint8_t __Int_BQ76952_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = BQ76952_CRC8_INIT;

    for (uint8_t i = 0u; i < len; i++)
    {
        crc = __Int_BQ76952_Crc8Update(crc, data[i]);
    }

    return crc;
}

/**
 * !BQ76952管理芯片 - 数据传输 - 发送直接指令 - 写
 */
BQ76952_StatusDef __Int_BQ76952_Send_Direct_Command_4_Write(uint16_t command, uint8_t *data, uint16_t size);
/**
 * !BQ76952管理芯片 - 数据传输 - 发送直接指令 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Send_Direct_Command_4_Read(uint16_t command, uint8_t *data, uint16_t size);
/**
 * !BQ76952管理芯片 - 数据传输 - 发送子指令
 */
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_Only_Command(uint16_t subcommand);
/**
 * !BQ76952管理芯片 - 数据传输 - 发送子指令 - 读
 */
//BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_With_Data(uint16_t command, uint8_t *data, uint16_t size);
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_4_Read(uint16_t command, uint8_t *data, uint16_t size);
/**
 * !BQ76952管理芯片 - 数据传输 - 发送子指令 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_4_Write(uint16_t command, uint8_t *data, uint16_t size);

/**
 * !BQ76952管理芯片 - 寄存器 - 写
 */
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write(uint16_t command, uint8_t *data, uint16_t size);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_8(uint16_t command, uint8_t data);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_16(uint16_t command, uint16_t data);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_32(uint16_t command, uint32_t data);
/**
 * !BQ76952管理芯片 - 寄存器 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read(uint16_t command, uint8_t *data, uint16_t size);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_8(uint16_t command, uint8_t *data);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_16(uint16_t command, uint16_t *data);
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_32(uint16_t command, uint32_t *data);
/**
 * BQ76952管理芯片 - 初始化
 */

/*
 * BQ APP 对外常量。
 * 只保留显示、CAN 或主机日志需要的值；内部调参项留在 App_BatMan.c。
 */
enum
{
    APP_BATMAN_CELL_COUNT = 6u,
    APP_BATMAN_CELL_FULL_MV = 4200u,
    APP_BATMAN_CELL_VALID_MIN_MV = 2500u,
    APP_BATMAN_CELL_VALID_MAX_MV = 4300u,
    APP_BATMAN_DEBUG_PERIOD_MS = 5000u,
    APP_BATMAN_CAPACITY_MAH = 5000u,
    APP_BATMAN_CC2_RAW_POLARITY = 1,
    APP_BATMAN_CC2_RAW_NUMERATOR = 1,
    APP_BATMAN_CC2_RAW_DENOMINATOR = 1,
    APP_BATMAN_STACK_RAW_TO_MV = 10u
};
/*
 * 显示 SOC 滤波系数。
 * 充电响应略快，放电响应略慢，静置时只轻微引入 OCV 校正，避免显示跳变。
 */
enum
{
    APP_BATMAN_SOC_REST_NEED_MS = 600000u,
    APP_BATMAN_SOC_CURRENT_DEADBAND_MA = 30,
    APP_BATMAN_SOC_REST_CURRENT_MA = 100,
    APP_BATMAN_SOC_REST_STABLE_MV = 3u,
    APP_BATMAN_SOC_FULL_CURRENT_MA = 250,
    APP_BATMAN_SOC_EMPTY_CURRENT_MA = 250,
    APP_BATMAN_SOC_FULL_ANCHOR_HOLD_MS = 5000u,
    APP_BATMAN_SOC_EMPTY_ANCHOR_HOLD_MS = 5000u,

    APP_BATMAN_RC_R0_MOHM = 35,
    APP_BATMAN_RC_RP_MOHM = 20,
    APP_BATMAN_RC_TAU_MS = 4000u,
    APP_BATMAN_RC_CURRENT_DEADBAND_MA = 200,
    APP_BATMAN_RC_COMP_MAX_MV = 550,

    APP_BATMAN_BALANCE_START_MIN_MV = 3900u,
    APP_BATMAN_BALANCE_STOP_MIN_MV = 3850u,
    APP_BATMAN_BALANCE_START_DELTA_MV = 40u,
    APP_BATMAN_BALANCE_STOP_DELTA_MV = 20u,
    APP_BATMAN_BALANCE_TEMP_MIN_C = 0,
    APP_BATMAN_BALANCE_TEMP_MAX_C = 45,
    APP_BATMAN_BALANCE_IC_TEMP_MAX_C = 70,

    /* 早期 SOH 趋势提醒，不是最终老化模型。 */
    APP_BATMAN_HEALTH_DELTA_WARN_MV = 80u,
    APP_BATMAN_HEALTH_TEMP_WARN_C = 55
};

#define APP_BATMAN_SOC_KALMAN_Q                 (0.002f)
#define APP_BATMAN_SOC_KALMAN_R                 (9.0f)
#define APP_BATMAN_SOC_OCV_STEP_LIMIT_PERCENT   (3.0f)
#define APP_BATMAN_SOC_DISPLAY_RISE_PER_S       (1.5f)
#define APP_BATMAN_SOC_DISPLAY_FALL_PER_S       (2.5f)
#define APP_BATMAN_DEFAULT_SOC_PERCENT           (50.0f)
void Int_BQ76952_Init(void)
{
    Com_SOC_ConfigTypeDef soc_config = {0};

    soc_config.capacity_mah = APP_BATMAN_CAPACITY_MAH;
    soc_config.default_soc_percent = APP_BATMAN_DEFAULT_SOC_PERCENT;
    soc_config.current_sign = COM_SOC_CURRENT_POS_CHARGE;
    soc_config.current_deadband_ma = APP_BATMAN_SOC_CURRENT_DEADBAND_MA;
    soc_config.rest_need_ms = APP_BATMAN_SOC_REST_NEED_MS;
    soc_config.rest_current_ma = APP_BATMAN_SOC_REST_CURRENT_MA;
    soc_config.rest_voltage_stable_mv = APP_BATMAN_SOC_REST_STABLE_MV;
    soc_config.kalman_q = APP_BATMAN_SOC_KALMAN_Q;
    soc_config.kalman_r = APP_BATMAN_SOC_KALMAN_R;
    soc_config.ocv_update_limit_percent = APP_BATMAN_SOC_OCV_STEP_LIMIT_PERCENT;
    soc_config.full_cell_mv = APP_BATMAN_CELL_FULL_MV;
    soc_config.empty_cell_mv = 3000u;
    soc_config.full_current_ma = APP_BATMAN_SOC_FULL_CURRENT_MA;
    soc_config.empty_current_ma = APP_BATMAN_SOC_EMPTY_CURRENT_MA;
    soc_config.full_anchor_hold_ms = APP_BATMAN_SOC_FULL_ANCHOR_HOLD_MS;
    soc_config.empty_anchor_hold_ms = APP_BATMAN_SOC_EMPTY_ANCHOR_HOLD_MS;
    soc_config.display_rise_percent_per_s = APP_BATMAN_SOC_DISPLAY_RISE_PER_S;
    soc_config.display_fall_percent_per_s = APP_BATMAN_SOC_DISPLAY_FALL_PER_S;
    Com_SOC_Init(&soc_config);
}

/**
 * BQ76952管理芯片 - 复位
 */
BQ76952_StatusDef Int_BQ76952_OP_Reset(void)
{
    BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_RESET);
    if (status != BQ76952_Status_OK)
    {
        if (status == BQ76952_Status_ERR_Send_Direct_CMD_NO_CRC)
        {
            s_bq76952_attribute.crc_status = BQ76952_CRC_Status_YES;
            status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_RESET);
        }
    }
    return status;
}
/**
 * BQ76952管理芯片 - 禁用睡眠
 */
BQ76952_StatusDef Int_BQ76952_OP_Disable_Sleep_Mode(void)
{
    return __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_SLEEP_DISABLE);
}

/**
 * BQ76952管理芯片 - 获取设备编号
 */
BQ76952_StatusDef Int_BQ76952_OP_Get_Device_Number(void)
{

    // 获取设备编号
    uint8_t data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_4_Read(BQ76952_SUBCMD_DEVICE_NUMBER, data, 2);
    if (status == BQ76952_Status_OK)
    {
        uint16_t device_number = (data[1] << 8) | data[0];
        printf("BQ76852 Device Number = %d \n", device_number);
    }
    return status;
}

/**
 * BQ76952管理芯片 - 修改具体每一项配置
 */
BQ76952_StatusDef __Int_BQ76952_OP_Config_Update_Item( uint8_t *name, uint8_t type, uint16_t command, uint32_t data)
{
    if ( type == 1 ) { // 8
        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint8_t old_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_8(command, &old_val);
            //printf("%s:old=%d,", name, old_val);
        }
        __Int_BQ76952_Data_Memory_4_Write_8(command, data);
        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint8_t new_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_8(command, &new_val);
            //printf("new=%d\n", new_val);
        }

    } else if ( type == 2 ) { // 16
        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint16_t old_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_16(command, &old_val);
            //printf("%s:old=%d,", name, old_val);
        }

        __Int_BQ76952_Data_Memory_4_Write_16(command, data);

        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint16_t new_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_16(command, &new_val);
            //printf("new=%d\n", new_val);
        }
    } else if ( type == 4 ) { //  32
        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint32_t old_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_32(command, &old_val);
            //printf("%s:old=%d,", name, old_val);
        }

        __Int_BQ76952_Data_Memory_4_Write_32(command, data);

        if ( s_bq76952_attribute.debug_status == BQ76952_Debug_Status_YES ) {
            uint32_t new_val = 0;
            __Int_BQ76952_Data_Memory_4_Read_32(command, &new_val);
            //printf("new=%d\n", new_val);
        }
    }

    return BQ76952_Status_OK;
}
/**
 * BQ76952管理芯片 - 修改配置
 */
BQ76952_StatusDef Int_BQ76952_OP_Config_Update(void)
{

    // 进入配置修改模式
    BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_SET_CFGUPDATE);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    // 修改配置
    __Int_BQ76952_OP_Config_Update_Item("DA Configuration", 1, BQ76952_DM_DA_CONFIGURATION, APP_BATMAN_DM_DA_CONFIGURATION_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Vcell Mode", 2, BQ76952_DM_VCELL_MODE, BQ76952_CELL_MASK_6S_HW);
    __Int_BQ76952_OP_Config_Update_Item("CC Gain", 4, BQ76952_DM_CC_GAIN, APP_BATMAN_DM_CC_GAIN_5_MOHM_IEEE754);
    __Int_BQ76952_OP_Config_Update_Item("Capacity Gain", 4, BQ76952_DM_CAPACITY_GAIN, APP_BATMAN_DM_CAPACITY_GAIN_5_MOHM_IEEE754);
    __Int_BQ76952_OP_Config_Update_Item("Protection Configuration", 2, BQ76952_DM_PROTECTION_CONFIGURATION, APP_BATMAN_DM_PROTECTION_CONFIGURATION_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Default Alarm Mask", 2, BQ76952_DM_DEFAULT_ALARM_MASK, APP_BATMAN_DM_DEFAULT_ALARM_MASK_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("CUV Threshold", 1, BQ76952_DM_CUV_THRESHOLD, APP_BATMAN_DM_CUV_THRESHOLD_2V83);
    __Int_BQ76952_OP_Config_Update_Item("CUV Delay", 2, BQ76952_DM_CUV_DELAY, APP_BATMAN_DM_CUV_DELAY_1S);
    __Int_BQ76952_OP_Config_Update_Item("CUV Recovery Hysteresis", 1, BQ76952_DM_CUV_RECOVERY_HYSTERESIS, APP_BATMAN_DM_CUV_RECOVERY_HYS_200MV);
    __Int_BQ76952_OP_Config_Update_Item("OCC Threshold", 1, BQ76952_DM_OCC_THRESHOLD, APP_BATMAN_DM_OCC_THRESHOLD_6A);
    __Int_BQ76952_OP_Config_Update_Item("OCC Delay", 1, BQ76952_DM_OCC_DELAY, APP_BATMAN_DM_OCC_DELAY_426MS);
    __Int_BQ76952_OP_Config_Update_Item("OCC Recovery Threshold", 2, BQ76952_DM_OCC_RECOVERY_THRESHOLD, APP_BATMAN_DM_OCC_RECOVERY_NEG_200MA);
    __Int_BQ76952_OP_Config_Update_Item("OCD1 Threshold", 1, BQ76952_DM_OCD1_THRESHOLD, APP_BATMAN_DM_OCD1_THRESHOLD_14A);
    __Int_BQ76952_OP_Config_Update_Item("OCD1 Delay", 1, BQ76952_DM_OCD1_DELAY, APP_BATMAN_DM_OCD1_DELAY_300MS);
    __Int_BQ76952_OP_Config_Update_Item("OCD2 Threshold", 1, BQ76952_DM_OCD2_THRESHOLD, APP_BATMAN_DM_OCD2_THRESHOLD_15A2);
    __Int_BQ76952_OP_Config_Update_Item("OCD2 Delay", 1, BQ76952_DM_OCD2_DELAY, APP_BATMAN_DM_OCD2_DELAY_80MS);
    __Int_BQ76952_OP_Config_Update_Item("SCD Threshold", 1, BQ76952_DM_SCD_THRESHOLD, APP_BATMAN_DM_SCD_THRESHOLD_80MV);
    __Int_BQ76952_OP_Config_Update_Item("SCD Delay", 1, BQ76952_DM_SCD_DELAY, APP_BATMAN_DM_SCD_DELAY_400US);
    __Int_BQ76952_OP_Config_Update_Item("OCD Recovery Threshold", 2, BQ76952_DM_OCD_RECOVERY_THRESHOLD, APP_BATMAN_DM_OCD_RECOVERY_500MA);
    __Int_BQ76952_OP_Config_Update_Item("OTC Threshold", 1, BQ76952_DM_OTC_THRESHOLD, APP_BATMAN_DM_OTC_THRESHOLD_50C);
    __Int_BQ76952_OP_Config_Update_Item("OTC Delay", 1, BQ76952_DM_OTC_DELAY, APP_BATMAN_DM_OTC_DELAY_3S);
    __Int_BQ76952_OP_Config_Update_Item("OTC Recovery", 1, BQ76952_DM_OTC_RECOVERY, APP_BATMAN_DM_OTC_RECOVERY_45C);
    __Int_BQ76952_OP_Config_Update_Item("OTD Threshold", 1, BQ76952_DM_OTD_THRESHOLD, APP_BATMAN_DM_OTD_THRESHOLD_60C);
    __Int_BQ76952_OP_Config_Update_Item("OTD Delay", 1, BQ76952_DM_OTD_DELAY, APP_BATMAN_DM_OTD_DELAY_3S);
    __Int_BQ76952_OP_Config_Update_Item("OTD Recovery", 1, BQ76952_DM_OTD_RECOVERY, APP_BATMAN_DM_OTD_RECOVERY_55C);
    __Int_BQ76952_OP_Config_Update_Item("FET Options", 1, BQ76952_DM_FET_OPTIONS, APP_BATMAN_DM_FET_OPTIONS_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Chg Pump Control", 1, BQ76952_DM_CHG_PUMP_CONTROL, APP_BATMAN_DM_CHG_PUMP_CONTROL_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Timeout", 1, BQ76952_DM_PREDISCHARGE_TIMEOUT, APP_BATMAN_DM_PREDISCHARGE_TIMEOUT_2500MS);
    __Int_BQ76952_OP_Config_Update_Item("Stop Delta", 1, BQ76952_DM_PREDISCHARGE_STOP_DELTA, APP_BATMAN_DM_PREDISCHARGE_STOP_DELTA_DISABLED);
    __Int_BQ76952_OP_Config_Update_Item("Balancing Configuration", 1, BQ76952_DM_BALANCING_CONFIGURATION, APP_BATMAN_DM_BALANCING_CONFIGURATION_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Enabled Protections A", 1, BQ76952_DM_ENABLED_PROTECTIONS_A, APP_BATMAN_DM_ENABLED_PROTECTIONS_A_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Enabled Protections B", 1, BQ76952_DM_ENABLED_PROTECTIONS_B, APP_BATMAN_DM_ENABLED_PROTECTIONS_B_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("Enabled Protections C", 1, BQ76952_DM_ENABLED_PROTECTIONS_C, APP_BATMAN_DM_ENABLED_PROTECTIONS_C_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("CHG FET Protections A", 1, BQ76952_DM_CHG_FET_PROTECTIONS_A, APP_BATMAN_DM_CHG_FET_PROTECTIONS_A_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("CHG FET Protections B", 1, BQ76952_DM_CHG_FET_PROTECTIONS_B, APP_BATMAN_DM_CHG_FET_PROTECTIONS_B_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("CHG FET Protections C", 1, BQ76952_DM_CHG_FET_PROTECTIONS_C, APP_BATMAN_DM_CHG_FET_PROTECTIONS_C_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("DSG FET Protections A", 1, BQ76952_DM_DSG_FET_PROTECTIONS_A, APP_BATMAN_DM_DSG_FET_PROTECTIONS_A_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("DSG FET Protections B", 1, BQ76952_DM_DSG_FET_PROTECTIONS_B, APP_BATMAN_DM_DSG_FET_PROTECTIONS_B_DEFAULT);
    __Int_BQ76952_OP_Config_Update_Item("DSG FET Protections C", 1, BQ76952_DM_DSG_FET_PROTECTIONS_C, APP_BATMAN_DM_DSG_FET_PROTECTIONS_C_DEFAULT);


    // 退出配置修改模式
    status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_EXIT_CFGUPDATE);
    return status;
}
/**
 * BQ76952管理芯片 - 清除启动告警
 */
BQ76952_StatusDef Int_BQ76952_OP_Clear_Startup_Alarm(void)
{
    uint16_t tx_data = 
        BQ76952_ALARM_FULLSCAN_MASK |
        BQ76952_ALARM_WAKE_MASK |
        BQ76952_ALARM_ADSCAN_MASK |
        BQ76952_ALARM_INITSTART_MASK |
        BQ76952_ALARM_INITCOMP_MASK
    ;
    
    uint8_t data[2] = {0};
    data[0] = tx_data & 0x00FF;
    data[1] = tx_data >> 8;
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_CMD_ALARM_STATUS, data, 2);
    return status;
}

/**
 * BQ76952管理芯片 - 设置FET模式
 */
BQ76952_StatusDef Int_BQ76952_OP_FET_Set_Mode(BQ76952_FET_Mode fet_mode) {

    // 临时读取数据缓冲区
    uint8_t tmp_data[2] = {0};
    // FET控制掩码
    uint8_t fet_mask = 0;

    uint8_t want_chg = 0;
    uint8_t want_dsg = 0;
    // Step 1 : 整合参数
    if ( fet_mode == BQ76952_FET_MODE_CHG ) {
        // 单独充电模式
        fet_mask = BQ76952_FET_CONTROL_PDSG_OFF_MASK | BQ76952_FET_CONTROL_DSG_OFF_MASK;
        want_chg = 1;
        want_dsg = 0;
    } else if ( fet_mode == BQ76952_FET_MODE_DSG ) {
        // 单独放电模式
        fet_mask = BQ76952_FET_CONTROL_CHG_OFF_MASK | BQ76952_FET_CONTROL_PCHG_OFF_MASK;
        want_chg = 0;
        want_dsg = 1;
    } else if ( fet_mode == BQ76952_FET_MODE_ALL_ON ) {
        // 充放电模式
        fet_mask = 0x00;
        want_chg = 1;
        want_dsg = 1;
    } else if ( fet_mode == BQ76952_FET_MODE_ALL_OFF ) {
        // 关闭充放电模式
        fet_mask = APP_BATMAN_MAIN_FET_OFF_MASK;
        want_chg = 0;
        want_dsg = 0;
    }
    printf("[FET] fet_mask = %#x, want_chg = %d, want_dsg = %d \n", fet_mask, want_chg, want_dsg);
    // Step 2 : 判断FET工作模式：不能为测试模式
    BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_4_Read(BQ76952_SUBCMD_MANUFACTURING_STATUS, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        printf("[FET] MANUFACTURING_STATUS Read Fail \n");
        return BQ76952_Status_ERR_FET_Fail;
    }
    uint16_t mf_status = tmp_data[1] << 8 | tmp_data[0];
    if ( (mf_status & BQ76952_MFG_STATUS_FET_EN_MASK ) == 0 ) {
        status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_FET_ENABLE); // Toggle
        if ( status != BQ76952_Status_OK ) {
            printf("[FET] FET_ENABLE Write Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }

        HAL_Delay(10);

        status = __Int_BQ76952_Send_SubCommand_4_Read(BQ76952_SUBCMD_MANUFACTURING_STATUS, tmp_data, 2);
        if ( status != BQ76952_Status_OK ) {
            printf("[FET] MANUFACTURING_STATUS Read Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }
        mf_status = tmp_data[1] << 8 | tmp_data[0];

        if ( (mf_status & BQ76952_MFG_STATUS_FET_EN_MASK ) == 0 ) {
            printf("[FET] FET_EN Status Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }
    }

    // Step 3 : 修改掩码 
    status = __Int_BQ76952_Send_SubCommand_4_Write(BQ76952_SUBCMD_FET_CONTROL, &fet_mask, 1);
    if ( status != BQ76952_Status_OK ) {
        printf("[FET] FET_CONTROL Write Fail \n");
        return BQ76952_Status_ERR_FET_Fail;
    }

    HAL_Delay(300);

    // Step 4 : 校验状态
    uint8_t fet_status;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_FET_STATUS, &fet_status, 1);
    if ( status != BQ76952_Status_OK ) {
        printf("[FET] FET_STATUS Read Fail \n");
        return BQ76952_Status_ERR_FET_Fail;
    }
    if (   ((want_chg && (fet_status & BQ76952_FET_STATUS_CHG_FET_MASK) == 0)) 
        || ((want_dsg && (fet_status & BQ76952_FET_STATUS_DSG_FET_MASK) == 0))
    ) {
        // 发送subcommand (such as 0x0096 ALL_FETS_ON())
        status = __Int_BQ76952_Send_SubCommand_Only_Command(BQ76952_SUBCMD_ALL_FETS_ON);
        if ( status != BQ76952_Status_OK ) {
            printf("[FET] ALL_FETS_ON Write Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }
        HAL_Delay(300);

        status = __Int_BQ76952_Send_SubCommand_4_Write(BQ76952_SUBCMD_FET_CONTROL, &fet_mask, 1);
        if ( status != BQ76952_Status_OK ) {
            printf("[FET] FET_CONTROL Write Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }
        if ( want_chg == 1 ) {
            HAL_Delay(300);
        }
        if ( want_dsg ) {
            HAL_Delay(APP_BATMAN_DM_PREDISCHARGE_TIMEOUT_2500MS * 10 + 500);
        }

        status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_FET_STATUS, &fet_status, 1);
        if ( status != BQ76952_Status_OK ) {
            printf("[FET] FET_STATUS Read Fail \n");
            return BQ76952_Status_ERR_FET_Fail;
        }

        if (((want_chg && (fet_status & BQ76952_FET_STATUS_CHG_FET_MASK) == 0))) {
            printf("[FET] FET_STATUS Bit1 Fail \n");
            return BQ76952_Status_ERR_FET_SW_Fail;
        }

        if (((want_dsg && (fet_status & BQ76952_FET_STATUS_DSG_FET_MASK) == 0))) {
            printf("[FET] FET_STATUS Bit2 Fail \n");
            return BQ76952_Status_ERR_FET_SW_Fail;
        }
    }

    // Step 5 : 非法验证
    if ( (want_chg == 0) && ( (fet_status & BQ76952_FET_STATUS_CHG_FET_MASK) != 0 ) ) {
        printf("[FET] FET_STATUS Bit3 Fail \n");
        return BQ76952_Status_ERR_FET_SW_Fail;
    }
    if ( (want_dsg == 0) && ( (fet_status & BQ76952_FET_STATUS_DSG_FET_MASK) != 0 ) ) {
        printf("[FET] FET_STATUS Bit4 Fail \n");
        return BQ76952_Status_ERR_FET_SW_Fail;
    }

    printf("[FET] FET_Set_Mode : Success!!!\n");
    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 关闭MOS管开关
 */
BQ76952_StatusDef Int_BQ76952_OP_Close_All_FET(void)
{
    return Int_BQ76952_OP_FET_Set_Mode(BQ76952_FET_MODE_ALL_OFF);
}

/**
 * BQ76952管理芯片 - 数据采样 - 采集电芯电压
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Cell_Voltage(void)
{
    s_sample.voltage.sample_valid = false;
    s_sample.voltage.cell_min_mv = 4200;
    s_sample.voltage.cell_max_mv = 0;
    s_sample.voltage.cell_avg_mv = 0;
    s_sample.voltage.cell_sum_mv = 0;
    s_sample.voltage.cell_delta_mv = 0;
    s_sample.voltage.cell_valid_count = 0;

    uint16_t cmds[6] = {
        BQ76952_CMD_CELL1_VOLTAGE,
        BQ76952_CMD_CELL2_VOLTAGE,
        BQ76952_CMD_CELL6_VOLTAGE,
        BQ76952_CMD_CELL9_VOLTAGE,
        BQ76952_CMD_CELL12_VOLTAGE,
        BQ76952_CMD_CELL16_VOLTAGE
    };
    BQ76952_StatusDef status;
    for (uint8_t i = 0; i < 6; i++)
    {
        uint8_t tmp_data[2] = {0};
        status = __Int_BQ76952_Send_Direct_Command_4_Read(cmds[i], tmp_data, 2);
        if ( status != BQ76952_Status_OK ) {
            continue;
        }
        uint16_t cell_voltage = (tmp_data[1] << 8) | (tmp_data[0]);
        if ( cell_voltage < 2500 || cell_voltage > 4300 ) {
            continue;
        }
        s_sample.voltage.vcell[i] = cell_voltage;
        s_sample.voltage.cell_sum_mv += cell_voltage;
        s_sample.voltage.cell_valid_count++;
        if ( s_sample.voltage.cell_max_mv < cell_voltage ) {
            s_sample.voltage.cell_max_mv = cell_voltage;
            s_sample.voltage.cell_max_index = i;
        }
        if ( s_sample.voltage.cell_min_mv > cell_voltage ) {
            s_sample.voltage.cell_min_mv = cell_voltage;
        }
        printf("[Sample] Cell Voltage : %dmV \n", cell_voltage); // 3.65V
    }

    if ( s_sample.voltage.cell_valid_count != 6 ) {
        printf("[Sample] Voltage Sample Fail : %d \n", s_sample.voltage.cell_valid_count); // 3.65V
        return BQ76952_Status_ERR;
    }

    s_sample.voltage.cell_avg_mv = s_sample.voltage.cell_sum_mv / s_sample.voltage.cell_valid_count;
    s_sample.voltage.cell_delta_mv = s_sample.voltage.cell_max_mv - s_sample.voltage.cell_min_mv;

    s_sample.voltage.sample_valid = true;
    printf("[Sample] Voltage Sample Success !!!\n");
    printf("[Sample] Voltage MAX = %d\n", s_sample.voltage.cell_max_mv);
    printf("[Sample] Voltage MIN = %d\n", s_sample.voltage.cell_min_mv);
    printf("[Sample] Voltage AVG = %d\n", s_sample.voltage.cell_avg_mv);
    printf("[Sample] Voltage SUM = %d\n", s_sample.voltage.cell_sum_mv);
    printf("[Sample] Voltage DELTA = %d\n", s_sample.voltage.cell_delta_mv);
    printf("[Sample] ********************************\n");
    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 数据采样 - 采集堆叠电压
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Stack_Voltage(void)
{
    uint8_t tmp_data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_STACK_VOLTAGE, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        return status;
    }
    uint16_t stack_voltage = (tmp_data[1] << 8) | (tmp_data[0]);
    s_sample.voltage.cell_stack_mv = stack_voltage * 10;
    printf("[Sample] Stack Voltage = %dmV \n", stack_voltage * 10);
    printf("[Sample] ***********************************\n");
    return BQ76952_Status_OK;
}

#define APP_BATMAN_CC2_LSB_MA          (1)      /* 01 档 = 1 mA/LSB */
#define APP_BATMAN_CC2_RAW_POLARITY    (+1)     /* 分流器方向反了改 -1 */
#define APP_BATMAN_CC2_RAW_NUMERATOR   (1000)   /* 增益分子 */
#define APP_BATMAN_CC2_RAW_DENOMINATOR (1000)   /* 增益分母 */

int32_t __Int_BQ76952_Scale_Current_Ma(int32_t raw_usera)
{
    int32_t denom = APP_BATMAN_CC2_RAW_DENOMINATOR;
    int64_t v;

    if (denom <= 0)                             /* 防除零、防负数分母 */
    {
        denom = 1;
    }

    v = (int64_t)raw_usera * APP_BATMAN_CC2_LSB_MA
      * APP_BATMAN_CC2_RAW_NUMERATOR
      * APP_BATMAN_CC2_RAW_POLARITY;

    /* 四舍五入（向零截断后补半格），int64 防中间结果溢出 */
    v = (v >= 0) ? (v + (int64_t)(denom / 2)) : (v - (int64_t)(denom / 2));

    return (int32_t)(v / denom);
}

/**
 * BQ76952管理芯片 - 数据采样 - 采集电流
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Current(void)
{
    s_sample.current.sample_valid = 0;
    uint8_t tmp_data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_CC2_CURRENT, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        return status;
    }
    uint16_t cc_current = (tmp_data[1] << 8) | (tmp_data[0]);
    int16_t cc_current_ma = __Int_BQ76952_Scale_Current_Ma(cc_current);
    float cc_current_a = cc_current_ma / 1000.0;
    printf("[Sample] Current CC2 = %dmA \n", cc_current_ma);
    printf("[Sample] ***********************************\n");
    s_sample.current.current_ma = cc_current_ma;
    s_sample.current.current_a = cc_current_a;
    s_sample.current.sample_valid = 1;
    return BQ76952_Status_OK;
}

/*
 * BQ76952 温度 direct command 单位是 0.1K。
 */
int16_t __Int_BQ76952_Temp_0p1K_2_C(int16_t temp_0p1k)
{
    int32_t temp_c_x10;

    temp_c_x10 = (int32_t)temp_0p1k - 2731;

    if (temp_c_x10 >= 0)
    {
        return (int16_t)((temp_c_x10 + 5) / 10);
    }

    return (int16_t)((temp_c_x10 - 5) / 10);
}

/**
 * BQ76952管理芯片 - 数据采样 - 采集温度
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Temperature(void)
{
    s_sample.temperature.sample_ic_valid = 0;
    s_sample.temperature.sample_ts1_valid = 0;
    s_sample.temperature.sample_ts3_valid = 0;
    s_sample.temperature.sample_cell_valid = 0;
    uint8_t tmp_data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_INT_TEMPERATURE, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        return status;
    }
    uint16_t u_int_temp = (tmp_data[1] << 8) | (tmp_data[0]);
    int16_t int_temp_c = __Int_BQ76952_Temp_0p1K_2_C(u_int_temp);
    s_sample.temperature.sample_ic_valid = 1;
    s_sample.temperature.ic = int_temp_c;
    s_sample.temperature.fet = int_temp_c;

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_TS1_TEMPERATURE, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        return status;
    }
    uint16_t u_ts1_temp = (tmp_data[1] << 8) | (tmp_data[0]);
    int16_t ts1_temp_c = __Int_BQ76952_Temp_0p1K_2_C(u_ts1_temp);
    if ( ts1_temp_c < -40 || ts1_temp_c > 85 ) {
        s_sample.temperature.sample_ts1_valid = 0;
    } else {
        s_sample.temperature.ts1 = ts1_temp_c;
        s_sample.temperature.sample_ts1_valid = 1;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_TS3_TEMPERATURE, tmp_data, 2);
    if ( status != BQ76952_Status_OK ) {
        return status;
    }
    uint16_t u_ts3_temp = (tmp_data[1] << 8) | (tmp_data[0]);
    int16_t ts3_temp_c = __Int_BQ76952_Temp_0p1K_2_C(u_ts3_temp);
    if ( ts3_temp_c < -40 || ts3_temp_c > 85 ) {
        s_sample.temperature.sample_ts3_valid = 0;
    } else {
        s_sample.temperature.ts3 = ts3_temp_c;
        s_sample.temperature.sample_ts3_valid = 1;
    }

    if ( s_sample.temperature.sample_ts1_valid && s_sample.temperature.sample_ts3_valid ) {
        s_sample.temperature.cell = (s_sample.temperature.ts1 + s_sample.temperature.ts3) / 2;
        s_sample.temperature.sample_cell_valid = 1;
    } else {
        if ( s_sample.temperature.sample_ts1_valid ) {
            s_sample.temperature.cell = s_sample.temperature.ts1;
            s_sample.temperature.sample_cell_valid = 1;
        } else if ( s_sample.temperature.sample_ts3_valid ) {
            s_sample.temperature.cell = s_sample.temperature.ts3;
            s_sample.temperature.sample_cell_valid = 1;
        } else {
            s_sample.temperature.cell = s_sample.temperature.ic;
            s_sample.temperature.sample_cell_valid = 0;
        }
    }

    // printf("[Sample] Temperature Int : %d \n", s_sample.temperature.ic);
    // printf("[Sample] Temperature TS1 : %d \n", s_sample.temperature.ts1);
    // printf("[Sample] Temperature TS3 : %d \n", s_sample.temperature.ts3);
    // printf("[Sample] ***********************************\n");

    return BQ76952_Status_OK;
}
/**
 * BQ76952管理芯片 - 数据采样 - 采集告警信息
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Alarm_Status(void)
{
    s_sample.alarm_status.sample_valid = 0;

    uint8_t data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_ALARM_STATUS, data, 2);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.alarm_status = data[1] << 8 | data[0];
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_ALARM_RAW_STATUS, data, 2);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.alarm_raw = data[1] << 8 | data[0];
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_BATTERY_STATUS, data, 2);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.battery_status = data[1] << 8 | data[0];
    }

    status = __Int_BQ76952_Send_SubCommand_4_Read(BQ76952_SUBCMD_MANUFACTURING_STATUS, data, 2);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.manufacturing_status = data[1] << 8 | data[0];
    }

    uint8_t data_8 = 0;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_FET_STATUS, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.fet_status = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_ALERT_A, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_alert_a = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_ALERT_B, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_alert_b = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_ALERT_C, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_alert_c = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_STATUS_A, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_status_a = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_STATUS_B, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_status_b = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_SAFETY_STATUS_C, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.safety_status_c = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_PF_STATUS_A, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.pf_status_a = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_PF_STATUS_B, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.pf_status_b = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_PF_STATUS_C, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.pf_status_c = data_8;
    }

    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_CMD_PF_STATUS_D, &data_8, 1);
    if (status == BQ76952_Status_OK)
    {
        s_sample.alarm_status.pf_status_d = data_8;
    }

    s_sample.alarm_status.sample_valid = 1;
    return BQ76952_Status_OK;
}

 /**
 * BQ76952管理芯片 - 更新全局错误状态
 */
BQ76952_StatusDef __Int_BQ76952_OP_Sample_Update_Fault_Status() {

    s_sample.alarm_status.global_fault = 0; // 没问题
    if ( s_sample.voltage.sample_valid == 0 ||
        s_sample.current.sample_valid == 0 ||
        s_sample.temperature.sample_cell_valid == 0
    ) {
        s_sample.alarm_status.global_fault = 1;
    } else {
        s_sample.alarm_status.global_fault = 0;
    }
    s_sample.alarm_status.global_cell_fault = s_sample.voltage.sample_valid;
    if ( s_sample.alarm_status.safety_status_a ||
         s_sample.alarm_status.safety_status_b ||
         s_sample.alarm_status.safety_status_c
    ) {
        s_sample.alarm_status.global_safety_fault = 1;
    } else {
        s_sample.alarm_status.global_safety_fault = 0;
    }

    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 数据采样
 */
BQ76952_StatusDef Int_BQ76952_OP_Sample(void)
{
    BQ76952_StatusDef status = __Int_BQ76952_OP_Sample_Cell_Voltage();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Sample_Cell_Voltage Fail \n");
        return status;
    }
    status = __Int_BQ76952_OP_Sample_Stack_Voltage();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Sample_Stack_Voltage Fail \n");
        return status;
    }
    status = __Int_BQ76952_OP_Sample_Current();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Sample_Current Fail \n");
        return status;
    }
    status = __Int_BQ76952_OP_Sample_Temperature();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Sample_Temperature Fail \n");
        return status;
    }
    status = __Int_BQ76952_OP_Sample_Alarm_Status();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Sample_Alarm_Status Fail \n");
        return status;
    }

    status = __Int_BQ76952_OP_Sample_Update_Fault_Status();
    if ( status != BQ76952_Status_OK ) {
        printf("[Sample] Update_Fault_Status Fail \n");
        return status;
    }

    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 数据传输 - 发送直接指令
 * 0x0012 => 12, CRC1, 00, CRC2
 * 2
 */
BQ76952_StatusDef __Int_BQ76952_Send_Direct_Command_4_Write(uint16_t command, uint8_t *data, uint16_t size)
{
    if (s_bq76952_attribute.crc_status == BQ76952_CRC_Status_NO)
    {
        // 不使用CRC进行IIC交互
        HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, BQ76952_I2C_8BIT_WRITE_ADDR_DEFAULT, command, I2C_MEMADD_SIZE_8BIT, data, size, BQ76952_I2C_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return BQ76952_Status_ERR_Send_Direct_CMD_NO_CRC;
        }
        else
        {
            return BQ76952_Status_OK;
        }
    }
    else
    {
        // 使用CRC进行IIC交互
        uint8_t data_buffer[1 + BQ76952_TRANSFER_BUFFER_SIZE * 2];
        // address + data1 + crc1 + data2 + crc2 ....
        data_buffer[0] = command;
        /**
         * 在块写入交易中，第一个数据字节的 CRC 是基于响应器地址、寄存器地址和数据计算得出的；
         * 后续数据字节的 CRC 则仅基于该数据字节计算得出
         */
        for (uint16_t i = 0; i < size; i++)
        {
            data_buffer[1 + i * 2] = data[i];
            uint8_t crc_data[3] = {0};
            if (i == 0)
            {
                crc_data[0] = BQ76952_I2C_8BIT_WRITE_ADDR_DEFAULT;
                crc_data[1] = command;
                crc_data[2] = data[i];
                data_buffer[1 + i * 2 + 1] = __Int_BQ76952_Crc8(crc_data, 3);
            }
            else
            {
                crc_data[0] = data[i];
                data_buffer[1 + i * 2 + 1] = __Int_BQ76952_Crc8(crc_data, 1);
            }
        }

        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, BQ76952_I2C_8BIT_WRITE_ADDR_DEFAULT, data_buffer, 1 + size * 2, BQ76952_I2C_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return BQ76952_Status_ERR_Send_Direct_CMD_CRC;
        }
        else
        {
            return BQ76952_Status_OK;
        }
    }
}
/**
 * BQ76952管理芯片 - 数据传输 - 发送子指令
 */
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_Only_Command(uint16_t subcommand)
{
    uint8_t sub_command[2] = {0};
    sub_command[0] = subcommand & 0x00FF;
    sub_command[1] = subcommand >> 8;
    return __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_SUBCMD_ADDR_LSB, sub_command, 2);
}
/**
 * BQ76952管理芯片 - 数据传输 - 发送子指令
    1. Write lower byte of subcommand to 0x3E.
    2. Write upper byte of subcommand to 0x3F.
    3. Read 0x3E and 0x3F. If this returns 0xFF, this indicates the subcommand has not completed operation
    yet. When the subcommand has completed, the readback will return what was originally written. Continue
    reading 0x3E and 0x3F until it returns what was written originally. Note: this response only applies to
    subcommands that return data to be read back.
    4. Read the length of response from 0x61.
    5. Read buffer starting at 0x40 for the expected length.
    6. Read the checksum at 0x60 and verify it matches the data read.
 */
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_4_Read(uint16_t subcommand, uint8_t *data, uint16_t size)
{
    // 1. Write lower byte of subcommand to 0x3E.
    // 2. Write upper byte of subcommand to 0x3F.
    uint8_t sub_command[2] = {0};
    sub_command[0] = subcommand & 0x00FF;
    sub_command[1] = subcommand >> 8;

    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_SUBCMD_ADDR_LSB, sub_command, 2);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    HAL_Delay(3);
    // 3. 读取指定位置的数据，判断数据有效性 (回响ECHO)
    uint8_t try_cnt = BQ76952_ECHO_POLL_COUNT;
    while (try_cnt)
    {
        uint8_t echo_data[2] = {0};
        status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_SUBCMD_ADDR_LSB, echo_data, 2);
        if (status == BQ76952_Status_OK)
        {
            if ((echo_data[0] == sub_command[0]) && (echo_data[1] == sub_command[1]))
            {
                break;
            }
        }
        try_cnt--;
        HAL_Delay(1);
    }

    if (try_cnt == 0)
    {
        return BQ76952_Status_ERR_Send_Sub_CMD_ECHO;
    }

    // 4. Read the length of response from 0x61. : 6 (2)
    uint8_t data_length = 0;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_LENGTH, &data_length, 1);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    // 5.  Read buffer starting at 0x40 for the expected length
    uint8_t data_buffer[BQ76952_TRANSFER_BUFFER_SIZE] = {0};
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_BUFFER_START, data_buffer, data_length - 4);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }

    // 6. Read the checksum at 0x60 and verify it matches the data read.
    uint8_t read_checksum = 0;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_CHECKSUM, &read_checksum, 1);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    // 校验和 = ~（子指令[0] + 子指令[1] + data1 + data2...）
    uint8_t calc_checksum = sub_command[0] + sub_command[1];
    for (uint16_t i = 0; i < data_length - 4; i++)
    {
        calc_checksum += data_buffer[i];
    }
    calc_checksum = ~calc_checksum;

    if (read_checksum != calc_checksum)
    {
        return BQ76952_Status_ERR_Send_Sub_CMD_CHECKSUM;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        data[i] = data_buffer[i];
    }

    return status;
}
/**
 * !BQ76952管理芯片 - 数据传输 - 发送子指令 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Send_SubCommand_4_Write(uint16_t command, uint8_t *data, uint16_t size) {

    uint8_t tx_data[4 + BQ76952_TRANSFER_MAX_LEN];
    tx_data[0] = command & 0x00FF;
    tx_data[1] = command >> 8;
    
    // 2 12, 00
    for (uint16_t i = 0; i < size; i++)
    {
        tx_data[2 + i] = data[i];
    }
    uint8_t check_sum = tx_data[0] + tx_data[1];
    for (uint16_t i = 0; i < size; i++)
    {
        check_sum += data[i];
    }
    check_sum = ~check_sum;
    tx_data[BQ76952_TRANSFER_MAX_LEN + 2] = check_sum;
    tx_data[BQ76952_TRANSFER_MAX_LEN + 3] = size + 4;
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_SUBCMD_ADDR_LSB, tx_data, 4 + BQ76952_TRANSFER_MAX_LEN);
    HAL_Delay(5);
    return status;
}
/**
 * !BQ76952管理芯片 - 数据传输 - 发送直接指令 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Send_Direct_Command_4_Read(uint16_t command, uint8_t *data, uint16_t size)
{
    if (s_bq76952_attribute.crc_status == BQ76952_CRC_Status_NO)
    {
        // 不使用CRC进行IIC交互
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, BQ76952_I2C_8BIT_READ_ADDR_DEFAULT, command, I2C_MEMADD_SIZE_8BIT, data, size, BQ76952_I2C_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return BQ76952_Status_ERR_Send_Direct_CMD_NO_CRC;
        }
        else
        {
            return BQ76952_Status_OK;
        }
    }
    else
    {
        // 使用CRC进行IIC交互
        // data1 -> crc1, data2 -> crc2
        uint8_t data_buffer[BQ76952_TRANSFER_MAX_LEN * 2];
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, BQ76952_I2C_8BIT_READ_ADDR_DEFAULT, command, I2C_MEMADD_SIZE_8BIT, data_buffer, size * 2, BQ76952_I2C_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            return BQ76952_Status_ERR_Send_Direct_CMD_CRC;
        }

        for (uint16_t i = 0; i < size; i++)
        {
            uint8_t rx_data = data_buffer[i * 2];
            uint8_t crc = data_buffer[i * 2 + 1];

            uint8_t crc_data[4] = {0};
            uint8_t new_crc = 0;
            if (i == 0)
            {
                // 包括应答器地址、寄存器地址，以及将读取位置为 1 后的应答器地址
                crc_data[0] = BQ76952_I2C_8BIT_WRITE_ADDR_DEFAULT >> 1;
                crc_data[1] = command;
                crc_data[2] = BQ76952_I2C_8BIT_READ_ADDR_DEFAULT;
                crc_data[3] = rx_data;

                new_crc = __Int_BQ76952_Crc8(crc_data, 4);
            }
            else
            {
                crc_data[0] = rx_data;
                new_crc = __Int_BQ76952_Crc8(crc_data, 1);
            }

            if (crc != new_crc)
            {
                return BQ76952_Status_ERR_Send_Direct_CMD_CRC;
            }

            data[i] = rx_data;
        }

        return BQ76952_Status_OK;
    }
}
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_8(uint16_t command, uint8_t data) {
    return __Int_BQ76952_Data_Memory_4_Write(command, &data, 1);
}
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_16(uint16_t command, uint16_t data) {
    uint8_t tx_data[2] = {0};
    tx_data[0] = data & 0x00FF;
    tx_data[1] = data >> 8;
    return __Int_BQ76952_Data_Memory_4_Write(command, tx_data, 2);
}
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write_32(uint16_t command, uint32_t data) {
    uint8_t tx_data[4] = {0};
    tx_data[0] = data & 0x00FF;
    tx_data[1] = data >> 8;
    tx_data[2] = data >> 16;
    tx_data[3] = data >> 24;
    return __Int_BQ76952_Data_Memory_4_Write(command, tx_data, 4);
}
/**
 * !BQ76952管理芯片 - 寄存器 - 写
 */
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Write(uint16_t command, uint8_t *data, uint16_t size) {
    uint8_t tx_data[4 + BQ76952_TRANSFER_MAX_LEN];
    tx_data[0] = command & 0x00FF;
    tx_data[1] = command >> 8;
    // 2 12, 00
    for (uint16_t i = 0; i < size; i++)
    {
        tx_data[2 + i] = data[i];
    }
    uint8_t check_sum = tx_data[0] + tx_data[1];
    for (uint16_t i = 0; i < size; i++)
    {
        check_sum += data[i];
    }
    check_sum = ~check_sum;
    tx_data[BQ76952_TRANSFER_MAX_LEN + 2] = check_sum;
    tx_data[BQ76952_TRANSFER_MAX_LEN + 3] = size + 4;
    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_SUBCMD_ADDR_LSB, tx_data, 4 + BQ76952_TRANSFER_MAX_LEN);
    HAL_Delay(5);
    return status;
}
/**
 * !BQ76952管理芯片 - 寄存器 - 读
 */
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read(uint16_t address, uint8_t *data, uint16_t size) {
    // 1. Write lower byte of subcommand to 0x3E.
    // 2. Write upper byte of subcommand to 0x3F.
    uint8_t sub_command[2] = {0};
    sub_command[0] = address & 0x00FF;
    sub_command[1] = address >> 8;

    BQ76952_StatusDef status = __Int_BQ76952_Send_Direct_Command_4_Write(BQ76952_SUBCMD_ADDR_LSB, sub_command, 2);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    HAL_Delay(3);
    // 3. 读取指定位置的数据，判断数据有效性 (回响ECHO)

    // 4. Read the length of response from 0x61. : 6 (2)
    uint8_t data_length = 0;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_LENGTH, &data_length, 1);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    
    // 5.  Read buffer starting at 0x40 for the expected length
    uint8_t data_buffer[BQ76952_TRANSFER_BUFFER_SIZE] = {0};
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_BUFFER_START, data_buffer, data_length - 4);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }

    // 6. Read the checksum at 0x60 and verify it matches the data read.
    uint8_t read_checksum = 0;
    status = __Int_BQ76952_Send_Direct_Command_4_Read(BQ76952_TRANSFER_CHECKSUM, &read_checksum, 1);
    if (status != BQ76952_Status_OK)
    {
        return status;
    }
    // 校验和 = ~（子指令[0] + 子指令[1] + data1 + data2...）
    uint8_t calc_checksum = sub_command[0] + sub_command[1];
    for (uint16_t i = 0; i < data_length - 4; i++)
    {
        calc_checksum += data_buffer[i];
    }
    calc_checksum = ~calc_checksum;

    if (read_checksum != calc_checksum)
    {
        return BQ76952_Status_ERR_Send_Sub_CMD_CHECKSUM;
    }

    for (uint16_t i = 0; i < size; i++)
    {
        data[i] = data_buffer[i];
    }

    return status;
}

BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_8(uint16_t command, uint8_t *data) {
    return __Int_BQ76952_Data_Memory_4_Read(command, data, 1);
}
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_16(uint16_t command, uint16_t *data) {
    uint8_t rx_data[2] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Data_Memory_4_Read(command, rx_data, 2);
    if ( status == BQ76952_Status_OK ) {
        *data = rx_data[1] << 8 | rx_data[0];
    }
    return status;
}
BQ76952_StatusDef __Int_BQ76952_Data_Memory_4_Read_32(uint16_t command, uint32_t *data) {
    uint8_t rx_data[4] = {0};
    BQ76952_StatusDef status = __Int_BQ76952_Data_Memory_4_Read(command, rx_data, 4);
    if ( status == BQ76952_Status_OK ) {
        *data = (rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0];
    }
    return status;
}

static int32_t __Int_BQ76952_OP_Limit_Rc_Mv(int32_t value_mv)
{
    if (value_mv < 0)
    {
        return 0;
    }
    if (value_mv > 500)
    {
        return 500;
    }

    return value_mv;
}
static uint16_t __Int_BQ76952_OP_Add_CellMv(uint16_t cell_raw_mv, int32_t comp_mv)
{
    uint32_t value_mv;

    if (cell_raw_mv == 0u)
    {
        return 0u;
    }

    value_mv = (uint32_t)cell_raw_mv + (uint32_t)__Int_BQ76952_OP_Limit_Rc_Mv(comp_mv);
    if (value_mv > 4300)
    {
        value_mv = 4300;
    }

    return (uint16_t)value_mv;
}

uint8_t __Int_BQ76952_RoundPercent(float value)
{
    if (value <= 0.0f)
    {
        return 0u;
    }
    if (value >= 100.0f)
    {
        return 100u;
    }
    return (uint8_t)(value + 0.5f);
}

/**
 * BQ76952管理芯片 - 更新RC电阻模型
 */
BQ76952_StatusDef Int_BQ76952_OP_Update_RC_Model(uint16_t interval_ms) {
    int32_t discharge_current_ma = 0;
    int32_t ohmic_mv = 0;
    int32_t polar_target_mv = 0;
    int32_t total_comp_mv;
    // 如果电压采集无效或电流采集无效，就采用默认值
    if (!s_sample.voltage.sample_valid || !s_sample.current.sample_valid)
    {
        s_sample.rc_model.cell_min_mv = s_sample.voltage.cell_min_mv;
        s_sample.rc_model.cell_avg_mv = s_sample.voltage.cell_max_mv;
        s_sample.rc_model.cell_ohmic_mv = 0;
        s_sample.rc_model.cell_polar_mv = (int16_t)__Int_BQ76952_OP_Limit_Rc_Mv(s_sample.rc_model.polar_mv);
        s_sample.rc_model.cell_total_mv = s_sample.rc_model.cell_polar_mv;
        return BQ76952_Status_OK;
    }

    if (s_sample.current.current_ma < -200)
    {
        discharge_current_ma = -s_sample.current.current_ma;
    }

    /*
     * Thevenin 一阶 RC 模型：实测端电压 = OCV - I*R0 - Vp。
     * 这里反推 OCV 估计值，只用于 APP 软欠压策略和日志，不替代 BQ 硬保护。
     * 
     * 欧姆内阻 35mΩ、极化内阻 20mΩ、极化时间常数 4 秒
     */
    // 计算电池内阻（欧姆内阻）降压
    ohmic_mv = (discharge_current_ma * 35) / 1000;
    // 计算电化学反应滞后带来的电压变化(极化内阻)
    polar_target_mv = (discharge_current_ma * 20) / 1000;

    if (interval_ms == 0u)
    {
        s_sample.rc_model.polar_mv = polar_target_mv;
    }
    else
    {
        s_sample.rc_model.polar_mv += ((polar_target_mv - s_sample.rc_model.polar_mv) * (int32_t)interval_ms) /
                         (int32_t)(4000 + interval_ms);
    }

    total_comp_mv = __Int_BQ76952_OP_Limit_Rc_Mv(ohmic_mv + s_sample.rc_model.polar_mv);
    s_sample.rc_model.cell_min_mv = __Int_BQ76952_OP_Add_CellMv(s_sample.voltage.cell_min_mv, total_comp_mv);
    s_sample.rc_model.cell_avg_mv = __Int_BQ76952_OP_Add_CellMv(s_sample.voltage.cell_avg_mv, total_comp_mv);
    s_sample.rc_model.cell_ohmic_mv = (int16_t)__Int_BQ76952_OP_Limit_Rc_Mv(ohmic_mv);
    s_sample.rc_model.cell_polar_mv = (int16_t)__Int_BQ76952_OP_Limit_Rc_Mv(s_sample.rc_model.polar_mv);
    s_sample.rc_model.cell_total_mv = (int16_t)total_comp_mv;

    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 更新电池容量百分比
 */
BQ76952_StatusDef Int_BQ76952_OP_Update_SOC(uint32_t interval_ms) {
    Com_SOC_SampleTypeDef sample = {0};
    Com_SOC_ResultTypeDef result;

    sample.soh_valid = 0;
    sample.interval_ms = interval_ms;
    sample.current_valid = s_sample.current.sample_valid;
    sample.current_ma = s_sample.current.current_ma;
    sample.cells_valid = s_sample.voltage.sample_valid;
    sample.cell_min_mv = s_sample.voltage.cell_min_mv;
    sample.cell_max_mv = s_sample.voltage.cell_max_mv;
    sample.cell_avg_mv = s_sample.voltage.cell_avg_mv;
    sample.temp_valid = s_sample.temperature.sample_cell_valid;
    sample.temp_c = s_sample.temperature.cell;
    printf("cell_avg_mv = %d \n", sample.cell_avg_mv);
    Com_SOC_Update(&sample);

    Com_SOC_GetResult(&result);
    s_sample.soc.percent = result.soc_percent;
    s_sample.soc.display_percent = result.display_percent;
    s_sample.soc.confidence_percent = result.confidence_percent;
    s_sample.soc.residual_percent = result.residual_percent;
    s_sample.soc.kalman_gain = result.kalman_gain_soc;
    s_sample.soc.p = result.p_soc;
    s_sample.soc.active_capacity_mah = result.active_capacity_mah;
    s_sample.soc.full_anchor_used = result.full_anchor_used;
    s_sample.soc.empty_anchor_used = result.empty_anchor_used;

    printf("percent = %f \n", s_sample.soc.percent);
    printf("display_percent = %d \n", __Int_BQ76952_RoundPercent(s_sample.soc.display_percent));
    printf("confidence_percent = %d \n", s_sample.soc.confidence_percent);
    printf("residual_percent = %f \n", s_sample.soc.residual_percent);

    return BQ76952_Status_OK;
}

/**
 * BQ76952管理芯片 - 获取SOC
 */
BQ76952_StatusDef Int_BQ76952_OP_Get_SOC(uint8_t init, uint8_t *soc) {

    if ( s_sample.alarm_status.global_fault == 1 ) {
        return BQ76952_Status_ERR;
    }

    if ( init == 1 ) {
        *soc = s_sample.soc.display_percent;
    } else {
        taskENTER_CRITICAL();
        *soc = s_sample.soc.display_percent;
        taskEXIT_CRITICAL();
    }

    return BQ76952_Status_OK;
}

void Int_BQ76952_OP_Get_Sample_Data(BQ76952_Sample *bq_sample) {
    *bq_sample = s_sample;
}
/**
 * BQ76952管理芯片 - 被动均衡（外部）
 */
BQ76952_StatusDef Int_BQ76952_OP_Cell_Balance(void) {

    // uint16_t balance_mask = BQ76952_VCELL_MODE_CELL1_MASK;
    // uint8_t data[2] = {0};
    // data[0] = balance_mask & 0x00FF;
    // data[1] = balance_mask >> 8;
    // __Int_BQ76952_Send_SubCommand_4_Write( BQ76952_SUBCMD_CB_ACTIVE_CELLS, data, 2);
    // // 20s
    // HAL_Delay(1000 * 60);
    // data[0] = 0;
    // data[1] = 0;
    // __Int_BQ76952_Send_SubCommand_4_Write( BQ76952_SUBCMD_CB_ACTIVE_CELLS, data, 2);

    // ! Step1 : 判断均衡周期
    // ! Step2 : 判断BQ芯片的有效性
    // ! Step3 : 根据实际情况，判断到底是那一节电芯应该做均衡
    // APP_BATMAN_BALANCE_START_MIN_MV = 3900u,
//     APP_BATMAN_BALANCE_STOP_MIN_MV = 3850u,
//     APP_BATMAN_BALANCE_START_DELTA_MV = 40u,
//     APP_BATMAN_BALANCE_STOP_DELTA_MV = 20u,
    uint32_t now = HAL_GetTick();
    if (  (now - s_bq76952_attribute.last_balance_tick ) < 1000 * 10  ) {
        return BQ76952_Status_OK;
    }

    s_bq76952_attribute.last_balance_tick = now;

    if (   s_sample.alarm_status.global_fault == 1
        || s_sample.alarm_status.global_safety_fault == 1
    ) {
        // 如果存在均衡的电芯，应该关闭
        if ( s_bq76952_attribute.last_balance_mask != 0 ) {

            uint8_t data[2] = {0, 0};
            BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_4_Write(BQ76952_SUBCMD_CB_ACTIVE_CELLS, data, 2);
            if ( status == BQ76952_Status_OK ) {
                s_bq76952_attribute.last_balance_mask = 0;
                s_bq76952_attribute.last_balance_index = 0xFF;
            }
            return status;
        }
        return BQ76952_Status_OK;
    }

    // !判断当前电压阈值和压差阈值
    uint16_t max_vol_th = 0;
    uint16_t delta_vol_th = 0;

    if ( s_bq76952_attribute.last_balance_mask == 0 ) {
        max_vol_th = APP_BATMAN_BALANCE_START_MIN_MV;
        delta_vol_th = APP_BATMAN_BALANCE_START_DELTA_MV;
    } else {
        max_vol_th = APP_BATMAN_BALANCE_STOP_MIN_MV;
        delta_vol_th = APP_BATMAN_BALANCE_STOP_DELTA_MV;
    }

    if (!(   s_sample.voltage.cell_max_mv > max_vol_th 
        &&   s_sample.voltage.cell_delta_mv > delta_vol_th )) {
        uint8_t data[2] = {0, 0};
        BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_4_Write(BQ76952_SUBCMD_CB_ACTIVE_CELLS, data, 2);
        if ( status == BQ76952_Status_OK ) {
            s_bq76952_attribute.last_balance_mask = 0;
            s_bq76952_attribute.last_balance_index = 0xFF;
        }
        return status;
    }

    // !配置均衡掩码
    uint16_t cells_mask[6] = {
        BQ76952_VCELL_MODE_CELL1_MASK ,
        BQ76952_VCELL_MODE_CELL2_MASK ,
        BQ76952_VCELL_MODE_CELL6_MASK ,
        BQ76952_VCELL_MODE_CELL9_MASK ,
        BQ76952_VCELL_MODE_CELL12_MASK,
        BQ76952_VCELL_MODE_CELL16_MASK
    };
    uint8_t index = s_sample.voltage.cell_max_index;
    uint16_t select_index = 0;
    if ( s_bq76952_attribute.last_balance_mask == 0 ) {
        select_index = index;
    } else if ( index == s_bq76952_attribute.last_balance_index ) {
        select_index = index;
    } else {
        uint16_t last_vol = s_sample.voltage.vcell[s_bq76952_attribute.last_balance_index];
        if ( (s_sample.voltage.cell_max_mv - last_vol) > 20 ) {
            // 切换均衡的电芯
            select_index = index;
        } else {
            // 不要切换均衡电芯
            select_index = s_bq76952_attribute.last_balance_index;
        }
    }

    // 需要均衡电芯掩码
    uint16_t mask = cells_mask[select_index];
    uint8_t data[2] = {0};
    data[0] = mask & 0x00FF;
    data[1] = mask >> 8;
    BQ76952_StatusDef status = __Int_BQ76952_Send_SubCommand_4_Write(BQ76952_SUBCMD_CB_ACTIVE_CELLS, data, 2);
    if ( status == BQ76952_Status_OK ) {
        s_bq76952_attribute.last_balance_mask = mask;
        s_bq76952_attribute.last_balance_index = select_index;
    }
    return status;
}
