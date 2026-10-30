#ifndef __QR_CODE_H
#define __QR_CODE_H

#include "main.h"      // 包含 uint8_t、uint16_t 定义
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "bsp_usart.h"
/* 四组原始顺序码 */
extern uint16_t openmv_shun_xu1;
extern uint16_t openmv_shun_xu2;
extern uint16_t openmv_shun_xu3;
extern uint16_t openmv_shun_xu4;

/* 第一组：范围 1~6 */
extern uint8_t A1;
extern uint8_t B1;
extern uint8_t C1;

/* 第二组：范围 1~3 */
extern uint8_t A2;
extern uint8_t B2;
extern uint8_t C2;

/* 第三组：范围 1~6 */
extern uint8_t A3;
extern uint8_t B3;
extern uint8_t C3;

/* 第四组：范围 1~3 */
extern uint8_t A4;
extern uint8_t B4;
extern uint8_t C4;

/* 当前实际抓取顺序 */
extern uint16_t A;
extern uint16_t B;
extern uint16_t C;

/* 二维码接收完成标志：1=未完成，0=已完成 */
extern volatile uint8_t E;

/* 接收到的四组三位数 */
extern volatile uint16_t ErWeiMaData[4];

/* 函数声明 */
uint8_t ShunXu_Decode(uint16_t shunxu,
                      uint8_t max_num,
                      uint8_t *A,
                      uint8_t *B,
                      uint8_t *C);

void USART2_IRQHandler_uaer(void);
void QRcode_print_1(void);

#endif
