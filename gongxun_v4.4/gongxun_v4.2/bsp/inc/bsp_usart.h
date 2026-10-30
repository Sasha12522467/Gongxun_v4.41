#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include  <stdio.h>
#include "stm32f4xx_hal.h"
#include "usart.h"
//#include "ring_buffer.h"
//#include "user_uart.h"
//#include "chassic.h"
//#include "string.h"
//#include "hwt101.h"
//#include "FSUS.h"
//#include "QR_code.h"
//#include "maixcam.h"

void uart_init_User(void);
void UART_SendbyteOne(UART_HandleTypeDef *huart,uint8_t *txbuf,uint16_t size);
void Usart_SendString(UART_HandleTypeDef *huart,uint8_t *str);
void EMM_ResReceiveToIdle_DMA(void);
void HWT101_ResReceiveToIdle_DMA(void);

#define EMM_RX_BUFFER_SIZE       16U
#define HWT101_RX_BUFFER_SIZE    64U
extern uint8_t RXbuf1,RXbuf5,RXbuf2,RXbuf3[EMM_RX_BUFFER_SIZE],RXbuf4,RXbuf6[HWT101_RX_BUFFER_SIZE];   //各个串口的数据缓冲区
#endif
