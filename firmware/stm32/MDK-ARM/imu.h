#ifndef _IMU229_H
#define _IMU229_H

#include "main.h"

void IMU_SendDebug(void); 
typedef struct {
    float pit;  // 俯仰角（对应H30的pitch）
    float rol;  // 横滚角（对应H30的roll）
    float yaw;  // 偏航角（对应H30的yaw）
} FLOAT_Angle;

#pragma pack(push, 1)
typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint16_t frame_id;
    uint8_t len;
} h30_header_t;

typedef struct {
    uint8_t data_id;
    uint8_t data_len;
    uint8_t data[];
} h30_payload_t;
#pragma pack(pop)

// 全局变量
extern FLOAT_Angle Angle_Measure;
extern uint8_t imu_data_ready;

// 陀螺仪漂移补偿相关
extern FLOAT_Angle imu_offset;  // 存储第一次有效数据的偏移量
extern uint8_t imu_offset_calibrated;  // 标志位：是否已经校准偏移量
void imu_reset_offset(void);  // 重置偏移量校准（可选功能）

// 函数声明
void h30_configure(void);
uint8_t h30_data_callback(uint8_t byte);
void h30_parse_data(uint8_t *data, uint16_t len);
float h30_bytes_to_float(uint8_t *bytes);

#endif
