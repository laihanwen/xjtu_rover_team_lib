#include "Move.h"
#include "math.h"
#include "RC.h"
MotorPower RCPower;
float RCStep,RaiseStep;
int L_Servo;

// ========== 分配矩阵 (Allocation Matrix) ==========
// 格式: [电机][控制指令]
// 控制指令: 0=上下, 1=左右平移, 2=前后, 3=左右转
// RC映射: RC[5]上升, RC[6]下降 | RC[7]左, RC[8]右 | RC[3]前进, RC[4]后退 | RC[1]左转, RC[2]右转
static const float allocation_matrix[8][4] = {
    {  1.0000f,   1.0000f,  -1.0000f,   1.0000f},  // Motor 1
    {  1.0000f,  -1.0000f,  -1.0000f,  -1.0000f},  // Motor 2
    {  1.0000f,   1.0000f,   1.0000f,  -1.0000f},  // Motor 3
    {  1.0000f,  -1.0000f,   1.0000f,   1.0000f},  // Motor 4
    { -1.0000f,   1.0000f,  -1.0000f,   1.0000f},  // Motor 5
    { -1.0000f,  -1.0000f,  -1.0000f,  -1.0000f},  // Motor 6
    { -1.0000f,   1.0000f,   1.0000f,  -1.0000f},  // Motor 7
    { -1.0000f,  -1.0000f,   1.0000f,   1.0000f},  // Motor 8
};

//����RcData[1]��������ת��RcData[2]����ǰ���ƶ���RcData[3]���������ƶ���RcData[4]�������Һ���
void RCPower_Calc(MotorPower* POWER, uint8_t *RC)
{
	//RCStep����������
	//ģʽ1������ģʽ
	if(RC[SC] == 0)
	{
		RCStep = RCStepA/0.35 * 0.6;	
	}
	
	//ģʽ 2������ģʽ
	if(RC[SC] == 1)
	{
		RCStep = RCStepA/0.35 * 0.6*1.2;	
	}
	
	//ģʽ3������ģʽ
	if(RC[SC] == 2)
	{
		RCStep = RCStepA/0.35 * 0.6*1.5;	
	}
	
	// 计算控制指令向量
	// 0=上下: RC[5]上升, RC[6]下降
	float command_up_down = (float)(RC[5] - RC[6]);
	// 1=左右平移: RC[7]左, RC[8]右
	float command_left_right = (float)(RC[7] - RC[8]);
	// 2=前后: RC[3]前进, RC[4]后退
	float command_forward_back = (float)(RC[3] - RC[4]);
	// 3=左右转: RC[1]左转, RC[2]右转
	float command_yaw = (float)(RC[1] - RC[2]);
	
	// 使用分配矩阵计算各电机功率
	POWER->MotorPow_1 = (allocation_matrix[0][0] * command_up_down + 
	                     allocation_matrix[0][1] * command_left_right + 
	                     allocation_matrix[0][2] * command_forward_back + 
	                     allocation_matrix[0][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_2 = (allocation_matrix[1][0] * command_up_down + 
	                     allocation_matrix[1][1] * command_left_right + 
	                     allocation_matrix[1][2] * command_forward_back + 
	                     allocation_matrix[1][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_3 = (allocation_matrix[2][0] * command_up_down + 
	                     allocation_matrix[2][1] * command_left_right + 
	                     allocation_matrix[2][2] * command_forward_back + 
	                     allocation_matrix[2][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_4 = (allocation_matrix[3][0] * command_up_down + 
	                     allocation_matrix[3][1] * command_left_right + 
	                     allocation_matrix[3][2] * command_forward_back + 
	                     allocation_matrix[3][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_5 = (allocation_matrix[4][0] * command_up_down + 
	                     allocation_matrix[4][1] * command_left_right + 
	                     allocation_matrix[4][2] * command_forward_back + 
	                     allocation_matrix[4][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_6 = (allocation_matrix[5][0] * command_up_down + 
	                     allocation_matrix[5][1] * command_left_right + 
	                     allocation_matrix[5][2] * command_forward_back + 
	                     allocation_matrix[5][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_7 = (allocation_matrix[6][0] * command_up_down + 
	                     allocation_matrix[6][1] * command_left_right + 
	                     allocation_matrix[6][2] * command_forward_back + 
	                     allocation_matrix[6][3] * command_yaw) * RCStep;
	
	POWER->MotorPow_8 = (allocation_matrix[7][0] * command_up_down + 
	                     allocation_matrix[7][1] * command_left_right + 
	                     allocation_matrix[7][2] * command_forward_back + 
	                     allocation_matrix[7][3] * command_yaw) * RCStep;		
	
	POWER->MotorPow_1 = POWER->MotorPow_1 * Motor_1Polarity;
	POWER->MotorPow_2 = POWER->MotorPow_2 * Motor_2Polarity;
	POWER->MotorPow_3 = POWER->MotorPow_3 * Motor_3Polarity;
	POWER->MotorPow_4 = POWER->MotorPow_4 * Motor_4Polarity;
	POWER->MotorPow_5 = POWER->MotorPow_5 * Motor_5Polarity;
	POWER->MotorPow_6 = POWER->MotorPow_6 * Motor_6Polarity;
	POWER->MotorPow_7 = POWER->MotorPow_7 * Motor_7Polarity;
	POWER->MotorPow_8 = POWER->MotorPow_8 * Motor_8Polarity;
	
}


void RCServo_Calc(uint8_t *RC){	
	int Servo_CCR,Servo = RC[9];	
	if(RC_WhetherSE_IN_JustNow() == FirstTime){
		L_Servo = Servo;
		RC_SI_Out = MyRCKey[9];
  }
	Servo = Servo_Limit(L_Servo + MyRCKey[9] - RC_SI_Out);
	if (RC[SB] == 0){ 
		Servo_CCR = 1100+(float)Servo/255.0f*(1900.0f-1100.0f);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Servo_CCR);
	}
	if (RC[SB] == 1){
		Servo_CCR = 1100+(float)Servo/255.0f*(1900.0f-1100.0f);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Servo_CCR);
	}	
	if (RC[SB] == 2){
		Servo_CCR = 1100+(float)Servo/255.0f*(1900.0f-1100.0f);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Servo_CCR);
	}		
}

int Servo_Limit(int a)
{
	if(a >= 255)
	{
		return 255;
	}
	else if(a <= 0)
	{
		return 0;
	}
	return a;
}
