/* protocol.h */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "main.h"

/* 协议帧格式 */
#define FRAME_HEADER        0xA5
#define FRAME_HEADER_TX     0x5A

/* 数据包结构 */
typedef struct {
    uint8_t header;
    uint8_t length;
    float yaw;
    float pitch;
    //uint8_t confidence; // 新增：置信度 0-100 (0=丢失, 100=锁定)
    uint8_t checksum;
} __attribute__((packed)) rx_frame_t;

typedef struct {
    uint8_t header;
    uint8_t length;
    float yaw;
    float pitch;
    uint8_t checksum;
} __attribute__((packed)) tx_frame_t;

uint8_t Protocol_Parse(uint8_t *data, uint32_t len, vision_target_t *target);
void Protocol_BuildTx(uint8_t *buffer, motor_feedback_t *yaw, motor_feedback_t *pitch);
uint8_t Protocol_CalcChecksum(uint8_t *data, uint32_t len);

#endif