/* Core/Src/gm6020.c */
#include "gm6020.h"
#include "can.h"
#include "gimbal_ctrl.h"


static motor_feedback_t yaw_motor = {0};
static motor_feedback_t pitch_motor = {0};


void GM6020_Init(void)
{
    /* 启动CAN接收 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void GM6020_SetCurrent(int16_t yaw_current, int16_t pitch_current)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t mailbox;
    
    /* 限幅 */
    if(yaw_current > CURRENT_MAX) yaw_current = CURRENT_MAX;
    if(yaw_current < CURRENT_MIN) yaw_current = CURRENT_MIN;
    if(pitch_current > CURRENT_MAX) pitch_current = CURRENT_MAX;
    if(pitch_current < CURRENT_MIN) pitch_current = CURRENT_MIN;
    
    /* 填充数据 - 标准6020协议 */
    tx_data[0] = (yaw_current >> 8) & 0xFF;
    tx_data[1] = yaw_current & 0xFF;
    tx_data[2] = (pitch_current >> 8) & 0xFF;
    tx_data[3] = pitch_current & 0xFF;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;
    
    tx_header.StdId = CAN_CMD_ID;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &mailbox);
}

/* CAN接收回调 - 需要在stm32f4xx_hal_msp.c或这里实现 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if(hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        
        switch(rx_header.StdId) {
            case YAW_FB_ID:
                yaw_motor.angle = (rx_data[0] << 8) | rx_data[1];
                yaw_motor.speed = (rx_data[2] << 8) | rx_data[3];
                yaw_motor.current = (rx_data[4] << 8) | rx_data[5];
                yaw_motor.temp = rx_data[6];
                break;
                
            case PITCH_FB_ID:
                pitch_motor.angle = (rx_data[0] << 8) | rx_data[1];
                pitch_motor.speed = (rx_data[2] << 8) | rx_data[3];
                pitch_motor.current = (rx_data[4] << 8) | rx_data[5];
                pitch_motor.temp = rx_data[6];
                break;
        }
    }
}

void GM6020_GetFeedback(motor_feedback_t *yaw, motor_feedback_t *pitch)
{
    *yaw = yaw_motor;
    *pitch = pitch_motor;
}

/* 角度转换函数 */
float GM6020_AngleToDegree(int16_t angle)
{
    return (float)angle * 360.0f / 8192.0f;
}

int16_t GM6020_DegreeToAngle(float degree)
{
    /* 处理角度环绕 */
    while(degree < 0) degree += 360;
    while(degree >= 360) degree -= 360;
    
    return (int16_t)(degree * 8192.0f / 360.0f);
}