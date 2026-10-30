#ifndef __MAIXCAM_H__
#define __MAIXCAM_H__

#include "stm32f4xx_hal.h"

#include "usart.h"
#include "bsp_usart.h"
#include "AIHMI.h"


#define 	maixcam_DATA_SIZE  10
/* 由串口接收中断更新，主任务读取时必须保留volatile。 */
extern volatile uint16_t YuanData[2];
extern volatile uint32_t maixcam_frame_count;
extern volatile uint32_t maixcam_last_update_ms;
extern uint8_t WuLiaoData,move_maix;
extern uint16_t yuanhuanx_last,yuanhuany_last;
void Maixcam_setmode(uint8_t mode);
void USART1_IRQHandler_uaer(void);
extern uint8_t mode1[3],mode2[3],mode3[3],mode4[3],mode5[3],mode6[3],mode7[3];
#endif
