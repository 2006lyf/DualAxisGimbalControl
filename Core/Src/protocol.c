/* protocol.c */
#include "protocol.h"
#include "gm6020.h"

uint8_t Protocol_CalcChecksum(uint8_t *data, uint32_t len)
{
    uint8_t sum = 0;
    for(uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

uint8_t Protocol_Parse(uint8_t *data, uint32_t len, vision_target_t *target)
{
    if(len < sizeof(rx_frame_t)) return 0;
    
    rx_frame_t *frame = (rx_frame_t*)data;
    
    /* 检查帧头 */
    if(frame->header != FRAME_HEADER) return 0;
    
    /* 校验和验证 */
    uint8_t cal_checksum = Protocol_CalcChecksum(data, sizeof(rx_frame_t) - 1);
    if(cal_checksum != frame->checksum) return 0;
    
    /* 解析数据 */
    target->yaw = frame->yaw;
    target->pitch = frame->pitch;
    target->confidence = 100;
    target->timestamp = HAL_GetTick();
    
    return 1;
}

void Protocol_BuildTx(uint8_t *buffer, imu_data_t *imu, motor_feedback_t *yaw, motor_feedback_t *pitch)
{
    tx_frame_t *frame = (tx_frame_t*)buffer;
    
    frame->header = FRAME_HEADER_TX;
    frame->length = sizeof(tx_frame_t) - 2;  // 除去header和checksum
    
    /* 当前角度 */
    frame->yaw = GM6020_AngleToDegree(yaw->angle);
    frame->pitch = GM6020_AngleToDegree(pitch->angle);
    frame->roll = 0;  // 单轴云台无roll
    
    /* 计算校验和 */
    frame->checksum = Protocol_CalcChecksum(buffer, sizeof(tx_frame_t) - 1);
}