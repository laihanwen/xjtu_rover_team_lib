#ifndef __MOVE_H
#define __MOVE_H

#include "tim.h"
#include "PID.h"
#include "gpio.h"

#define Motor_1Polarity  -1//1或-1(控制电机正反转的）
#define Motor_2Polarity  1
#define Motor_3Polarity  -1
#define Motor_4Polarity  1

#define Motor_8Polarity   -1
#define Motor_5Polarity   -1
#define Motor_7Polarity   -1
#define Motor_6Polarity   -1
#define deadzone 10

#define Servoangstart  47    //每次把机械臂撑大的时候就把这个数值调小
#define MAXServoAngle 60
#define startendang MAXServoAngle+Servoangstart

#define Servo_C_MAX 100
#define Servo_C_Start 60

#define Servo_Injector_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define Servo_Injector_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)


//ROV的运动只有两个平面，一个是水平面，另一个时ROLL的垂直平面
//而在这两个平面中，都有三个运动
//水平面: 1.遥控器控制的前后左右四个方向的运动
//        2.遥控器控制的Yaw方向旋转
//        3.PID控制的YAW方向的自稳

//垂直平面: 1.遥控器控制的上下运动
//          2.遥控器控制的ROLL运动
//          3.PID控制的ROLL

//可见在这两个平面中，1与2.3都没有重叠，所以在任何时候1都由遥控器控制，而2和3在遥控器不控制时由PID接管
//YAW的PID的输入是遥控器停止控制的那一刻的yAW的方向（正好由于陀螺仪YAW方向的飘移比较大，所以尽管有漂移，但是时间比较短，也可以达到精确控制）
//本来想采用动态的动力分配，但懒得搞，就直接给1分配50%的动力，2，3分配30%的动力（2.3作用时间不重叠，所以总动力不会超过100%）
//动态的分配方法就是1与2，3加起来等于0.8，但各自没有限额，也就是说在无2，3时1最大可达80%


#define RCPowerMAX 0.35f
#define RP_PIDPowerMAX 0.25f
#define Y_PIDPowerMAX 0.3f
#define SpinPowerMAX 0.30f

#define RaisePowerMAX 0.4f//上升下降专门的
#define PowerMAXpercent  0.9f

#define RCMAX  255.0f//按键可传输的最大值

#define CCRSingleLimit 490.0f //意思就是CCR的值是从中值最大可以上升或者下降的范围

#define RCStepA   (RCPowerMAX * (CCRSingleLimit / RCMAX))//750是PWM的范围是1500-3000，2250是中值
#define Y_PIDStep   (CCRSingleLimit * Y_PIDPowerMAX)//记得除以PID_OUTimit
#define RP_PIDStep   (CCRSingleLimit * RP_PIDPowerMAX)//记得除以PID_OUTimit
#define SpinStep   (SpinPowerMAX * (CCRSingleLimit / RCMAX))
#define RaiseStepA   (RaisePowerMAX * (CCRSingleLimit / RCMAX))

typedef struct
{
	float	MotorPow_1;
	float	MotorPow_2;
	float	MotorPow_3;
	float	MotorPow_4;
		
	float	MotorPow_5;
	float	MotorPow_6;
	float	MotorPow_7;
	float	MotorPow_8;
	
}MotorPower;//各个电机的动力
void RCPower_Calc(MotorPower* POWER, uint8_t *RC);
void RCServo_Calc(uint8_t *RC);
int Servo_Limit(int a);
#endif
