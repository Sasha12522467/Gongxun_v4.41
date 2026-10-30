#ifndef __HWT101_H__
#define __HWT101_H__

#include "stm32f4xx_hal.h"

typedef struct {
	float yaw_deg;
	uint32_t frame_count;
	uint32_t last_update_ms;
} HWT101_Snapshot_t;

extern volatile float yaw;    //陀螺仪的角度
float imu_pid(float hope);
void PID_Init(void);
void USART6_IRQHandler_uaer(uint16_t size);
void Hwt101_Yaw_Res(void);
void hwt101_JiaoZun(void);
void hwt101_Get(void);
void HWT101_GetSnapshot(HWT101_Snapshot_t *snapshot);
uint8_t HWT101_IsOnline(uint32_t timeout_ms);
#endif
