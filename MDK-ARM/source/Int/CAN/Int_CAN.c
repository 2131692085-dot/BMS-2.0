#include "Int_CAN.h"
#include "fdcan.h"

#define CAN_ID_BMS 1001
#define CAN_ID_GATEWAY 1002

/**
 * CAN通信 - 初始化
 */
void Int_CAN_Init(void) {


    // 配置过滤器
    FDCAN_FilterTypeDef fdcan_filter_config;
    fdcan_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter_config.FilterID1 = CAN_ID_GATEWAY;
    fdcan_filter_config.FilterID2 = CAN_ID_GATEWAY;
    fdcan_filter_config.FilterIndex = 0;
    fdcan_filter_config.FilterType = FDCAN_FILTER_DUAL;
    fdcan_filter_config.IdType = FDCAN_STANDARD_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter_config);

    // 启动FDCAN
    HAL_FDCAN_Start(&hfdcan1);
}

/**
 * CAN通信 - 发送数据
 */
void Int_CAN_Send_Data(uint8_t *data, uint16_t length) {

    while (1) {
        uint32_t free_mail_box_num = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        if ( free_mail_box_num > 0 ) {
            break;
        }
        HAL_Delay(1);
    }

    FDCAN_TxHeaderTypeDef can_message_header;
    can_message_header.BitRateSwitch = FDCAN_BRS_OFF;
    can_message_header.DataLength = length;
    can_message_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can_message_header.FDFormat = FDCAN_CLASSIC_CAN;
    can_message_header.Identifier = CAN_ID_BMS;
    can_message_header.IdType = FDCAN_STANDARD_ID;
    can_message_header.MessageMarker = 0;
    can_message_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    can_message_header.TxFrameType = FDCAN_DATA_FRAME;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_message_header, data);
}

/**
 * CAN通信 - 接收数据
 */
void Int_CAN_Recevie_Data(CAN_Message *message, uint32_t *message_size) {

    while (1) {
        *message_size = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
        if ( *message_size > 0 ) {
            break;
        }
        HAL_Delay(1);
    }

    for (uint8_t i = 0; i < *message_size; i++)
    {
        FDCAN_RxHeaderTypeDef can_message_header;
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_message_header, message[i].data);
        message[i].data_length = can_message_header.DataLength;
        message[i].stdid = can_message_header.Identifier;
    }
    

    
}
