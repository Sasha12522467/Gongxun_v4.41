#ifndef CHASSIC_H
#define CHASSIC_H
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Emm_V5.h"
#include "bsp_usart.h"
#include "hwt101.h"

#define ML_Up        			1 
#define ML_UpLeft        	2 
#define ML_Left        		3 
#define ML_DownLeft       4 
#define ML_Down        		5
#define ML_DownRight      6 
#define ML_Right        	7
#define ML_UpRight        8

#define ML_TurnLeft       9 
#define ML_TurnRight      10
extern uint16_t path;
extern uint8_t DataBufs;
extern uint8_t CmdFlagTx;   
extern uint8_t CmdFlagPulse;
void Edge_path(int path);
void MotorSetVel(uint8_t Crl, uint16_t vel, uint16_t acc);
void MotorSetPulse(uint32_t pulse, uint8_t Crl, uint16_t vel, uint16_t acc);
void motor_mildage_set(uint32_t pulse,int x,int y ,float z,uint16_t acc);
void motor_Omnidirectional_set(int x,int y,float z,uint16_t acc);
static int Emm_data_2_parse(uint8_t *pack);
void USART3_IRQHandler_uaer(void);
void Angle_correc(float z2);
#endif // MOTOR_OMNIDIRECTIONAL_H

