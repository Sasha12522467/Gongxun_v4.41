#include "bsp_usart.h"
#include "stm32f4xx_hal.h"
#include "user_uart.h"    //串口舵机
#include "QR_code.h"    //扫码
#include "chassic.h"   //底盘-张大头
#include "maixcam.h"    //摄像头
#include "radar_v6.h"   //雷达


extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart5;
//extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart6;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern uint8_t rc1;//接收中断缓存  串口舵机  环缓

uint8_t RXbuf1,RXbuf5,RXbuf2,RXbuf3[EMM_RX_BUFFER_SIZE],RXbuf4,RXbuf6[HWT101_RX_BUFFER_SIZE];   //各个串口的数据缓冲区

/**
  * @brief UART重定向
  * @param 
  * @retval 无
  */

//int fputc(int ch, FILE *fp)
//{
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}
// 
//int fgetc(FILE *fp)
//{
//  uint8_t ch = 0;
//  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
//  return ch;
//}

/**
  * @brief UART 初始化 
  * @param huart: UART handle
  * @retval 无
  */

void EMM_ResReceiveToIdle_DMA(void)
{
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RXbuf3, sizeof(RXbuf3)); //接收完毕后重启
	  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);		                  //手动关闭DMA_IT_HT中断
}

void HWT101_ResReceiveToIdle_DMA(void)
{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, RXbuf6, sizeof(RXbuf6)); //接收完毕后重启
	  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);		                  //手动关闭DMA_IT_HT中断
}

void uart_init_User()
{
	
	HAL_UART_Receive_IT(&huart1,&RXbuf1,1);           //摄像头 		--普通中端
	HAL_UART_Receive_IT(&huart2,&RXbuf2,1);           //扫码模块  --普通中端
	//	HAL_UART_Receive_IT(&huart3,&RXbuf3,1);      //ZDT电机    --轮询模式-无需开启中断
//	HAL_UART_Receive_IT(&huart4,&RXbuf4,1);          //串口舵机 --dma中端
//	HAL_UART_Receive_IT(&huart5,&RXbuf5,1);	         //激光雷达 --内部代码开启-通用中断无需开启
}

/**
  * @brief UART的单个或多个字节发送函数
  * @param huart: UART handle  TXBuf   size
  * @retval 无
  */

void UART_SendbyteOne(UART_HandleTypeDef *huart,uint8_t *txbuf,uint16_t size)
{
	HAL_UART_Transmit(huart,txbuf,size,0xff);
}

/**
  * @brief UART的字符串发送函数
  * @param huart: UART handle  str
  * @retval 无
  */
void Usart_SendString(UART_HandleTypeDef *huart,uint8_t *str)
{
	unsigned int k=0;
  do 
  {
      HAL_UART_Transmit(huart,(uint8_t *)(str + k) ,1,1000);
      k++;
  } while(*(str + k)!='\0');
}

/************************   串口中断     ***************************/
int a=0,b=0,c=0,d=0,e=0;

//接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//usart1
	uint8_t ucTemp;
	if(huart==&huart4)          //串口舵机
	{
		a++;
		ucTemp = rc1;
		RingBuffer_Push(FSUS_Usart.recvBuf, ucTemp);// 接收到数据放入缓冲区，不在中断具体处理数据
		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
		
	}
	else if(huart->Instance == UART5)
  {
		b++;
  }
	else if(huart==&huart2)   
	{
		c++;
		USART2_IRQHandler_uaer();
	}
	else if(huart==&huart1)
	{
		e++;
		USART1_IRQHandler_uaer();
	}	
	HAL_UART_Receive_IT(&huart2,&RXbuf2,1);
	HAL_UART_Receive_IT(&huart1,&RXbuf1,1);
	
}

//事件中断
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART6)     //HWT101陀螺仪
	{
   USART6_IRQHandler_uaer(Size);
	}
}

//错误中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
	}
	else if(huart->Instance==USART2)
	{
		HAL_UART_Receive_IT(&huart2,&RXbuf2,1);
	}	
  else  if(huart->Instance==USART3)
	{
		
	}
	else if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,&RXbuf1,1);
	}
   else if (huart->Instance == UART5)
   {
     RadarClient_OnUartErrorInterrupt(huart->ErrorCode);
   }
  else if(huart->Instance==USART6)
	{
		HWT101_ResReceiveToIdle_DMA();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
        RadarClient_OnUartTxCompleteInterrupt();
}
