#include "chassic.h"
#include "chassis_direction.h"
#include "main.h"
#include "Emm_V5.h"
#include "hwt101.h"
#include "string.h"
#include "cmsis_os.h"
void MotorSetVel(uint8_t Crl, uint16_t vel, uint16_t acc)
{
	char CW[4];
	if(Crl==1) 			{CW[0]=1;CW[1]=0;CW[2]=0;CW[3]=1;}
	else if(Crl==2) {CW[0]=1;CW[1]=0;CW[2]=0;CW[3]=1;}
	else if(Crl==3) {CW[0]=1;CW[1]=1;CW[2]=0;CW[3]=0;}
	else if(Crl==4)	{CW[0]=1;CW[1]=1;CW[2]=1;CW[3]=0;}
	else if(Crl==5)	{CW[0]=0;CW[1]=1;CW[2]=1;CW[3]=0;}
	else if(Crl==6)	{CW[0]=0;CW[1]=1;CW[2]=1;CW[3]=0;}
	else if(Crl==7)	{CW[0]=0;CW[1]=0;CW[2]=1;CW[3]=1;}
	else if(Crl==8)	{CW[0]=0;CW[1]=0;CW[2]=0;CW[3]=1;}
	else if(Crl==9)	{CW[0]=1;CW[1]=1;CW[2]=1;CW[3]=1;}     //逆时针
	else if(Crl==10)	{CW[0]=0;CW[1]=0;CW[2]=0;CW[3]=0;}   //顺时针
	
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M0_ADDRESS, CW[0], vel, acc,  true);
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M1_ADDRESS, CW[1], vel, acc,  true);
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M2_ADDRESS, CW[2], vel, acc, 	true);
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M3_ADDRESS, CW[3], vel, acc,  true);
	HAL_Delay(1);
	Emm_V5_Synchronous_motion(0);
}

//位置模式
void MotorSetPulse(uint32_t pulse, uint8_t Crl, uint16_t vel, uint16_t acc)
{
	char CW[8];
	if(Crl==1)  //
	{
		CW[0]=1;	CW[4]=1;
		CW[1]=0;	CW[5]=1;
		CW[2]=0;	CW[6]=1;
		CW[3]=1;	CW[7]=1;
	}
	else if(Crl==2)  //
	{
		CW[0]=1;	CW[4]=1;
		CW[1]=0;	CW[5]=0;
		CW[2]=0;	CW[6]=1;
		CW[3]=1;	CW[7]=0;
	}
	else if(Crl==3)  //
	{
		CW[0]=1;	CW[4]=1;
		CW[1]=1;	CW[5]=1;
		CW[2]=0;	CW[6]=1;
		CW[3]=0;	CW[7]=1;
	}
	else if(Crl==4)  //
	{
		CW[0]=1;	CW[4]=0;
		CW[1]=1;	CW[5]=1;
		CW[2]=1;	CW[6]=0;
		CW[3]=0;	CW[7]=1;
	}
	else if(Crl==5)  //
	{
		CW[0]=0;	CW[4]=1;
		CW[1]=1;	CW[5]=1;
		CW[2]=1;	CW[6]=1;
		CW[3]=0;	CW[7]=1;
	}
	else if(Crl==6)  //
	{
		CW[0]=0;	CW[4]=1;
		CW[1]=1;	CW[5]=0;
		CW[2]=1;	CW[6]=1;
		CW[3]=0;	CW[7]=0;
	}
	else if(Crl==7)  //
	{
		CW[0]=0;	CW[4]=1;
		CW[1]=0;	CW[5]=1;
		CW[2]=1;	CW[6]=1;
		CW[3]=1;	CW[7]=1;
	}
	else if(Crl==8)  //
	{
		CW[0]=0;	CW[4]=0;
		CW[1]=0;	CW[5]=1;
		CW[2]=0;	CW[6]=0;
		CW[3]=1;	CW[7]=1;
	}
	else if(Crl==9)  //
	{
		CW[0]=1;	CW[4]=1;
		CW[1]=1;	CW[5]=1;
		CW[2]=1;	CW[6]=1;
		CW[3]=1;	CW[7]=1;
	}
	else if(Crl==10)  //
	{
		CW[0]=0;	CW[4]=1;
		CW[1]=0;	CW[5]=1;
		CW[2]=0;	CW[6]=1;
		CW[3]=0;	CW[7]=1;
	}

	Emm_V5_Pos_Control(CHASSIS_MOTOR_M0_ADDRESS, CW[0], vel, acc, pulse*CW[4], false, true);
	osDelay(2);
	Emm_V5_Pos_Control(CHASSIS_MOTOR_M1_ADDRESS, CW[1], vel, acc, pulse*CW[5], false, true);
	osDelay(2);
	Emm_V5_Pos_Control(CHASSIS_MOTOR_M2_ADDRESS, CW[2], vel, acc, pulse*CW[6], false, true);
	osDelay(2);
	Emm_V5_Pos_Control(CHASSIS_MOTOR_M3_ADDRESS, CW[3], vel, acc, pulse*CW[7], false, true);
	osDelay(2);
	Emm_V5_Synchronous_motion(0);//多机同步运动控制
}
int my_abs(float num) 
{
    return (num < 0) ? -num : num;
}

//全向运动   1---正  方向
void motor_Omnidirectional_set(int x,int y,float z,uint16_t acc)
{
	static float motor1,motor2,motor3,motor4;
	static float z1=0;
	float forward = (float)x * (float)CHASSIS_FORWARD_SIGN;
	float lateral = (float)y * (float)CHASSIS_LATERAL_SIGN;
	char CW_1[4];CW_1[0]=1;CW_1[1]=1;CW_1[2]=1;CW_1[3]=1;

	z1=imu_pid(z);
	if(my_abs(z)<0.4)z=0;
	
	/* 视觉微调与正式导航共用平移方向校准，航向PID输出不反向。 */
	motor1=forward+lateral+z1;
	motor2=-forward+lateral+z1;
	motor3=-forward-lateral+z1;
	motor4=forward-lateral+z1;

	if(motor1<0)CW_1[0]=0;
	if(motor2<0)CW_1[1]=0;
	if(motor3<0)CW_1[2]=0;
	if(motor4<0)CW_1[3]=0;
	
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M0_ADDRESS, CW_1[0], (uint16_t)my_abs(motor1), acc,  true);  
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M1_ADDRESS, CW_1[1], (uint16_t)my_abs(motor2), acc,  true);
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M2_ADDRESS, CW_1[2], (uint16_t)my_abs(motor3), acc,  true);
	HAL_Delay(1);
	Emm_V5_Vel_Control(CHASSIS_MOTOR_M3_ADDRESS, CW_1[3], (uint16_t)my_abs(motor4), acc,  true);
	HAL_Delay(1);
	Emm_V5_Synchronous_motion(0);
}

