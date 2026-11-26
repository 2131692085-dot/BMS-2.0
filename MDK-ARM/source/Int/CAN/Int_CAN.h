#ifndef __INT_CAN_H__
#define __INT_CAN_H__

#include "stm32g0xx_hal.h"

typedef struct
{
    uint8_t data[8];
    uint8_t data_length;
    uint32_t stdid;
} CAN_Message;


/**
 * CAN通信 - 初始化
 */
void Int_CAN_Init(void);

/**
 * CAN通信 - 发送数据
 */
void Int_CAN_Send_Data(uint8_t *data, uint16_t length);

/**
 * CAN通信 - 接收数据
 */
void Int_CAN_Recevie_Data(CAN_Message *message, uint32_t *message_size);

#endif /* __INT_CAN_H__ */
