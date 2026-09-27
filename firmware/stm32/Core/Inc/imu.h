#ifndef _IMU229_H
#define _IMU229_H

#include "main.h"
typedef struct
{ 
	float rol;
	float pit;
	float yaw;
} FLOAT_Angle;

typedef struct
{
	float x;
	float y;
	float z;
}FLOAT_xyz;

#define imu_length 80 
//��33->11��
extern FLOAT_Angle Angle_Measure;
//extern FLOAT_Angle AngleRate_Measure;
//extern FLOAT_xyz   Gry_Measure;
//��������û�õ�����
extern uint8_t imu229;
extern uint8_t imu229_Data[imu_length];
void imu229_Translate(uint8_t data[],FLOAT_Angle *Angle);
void imu229_Get(void);

// 陀螺仪漂移补偿相关
extern FLOAT_Angle imu_offset;  // 存储第一次有效数据的偏移量
extern uint8_t imu_offset_calibrated;  // 标志位：是否已经校准偏移量
void imu_reset_offset(void);  // 重置偏移量校准（可选功能）

#endif
