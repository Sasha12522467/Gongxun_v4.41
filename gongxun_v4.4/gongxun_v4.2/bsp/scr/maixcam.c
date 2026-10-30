#include "maixcam.h"

volatile uint16_t YuanData[2];
volatile uint32_t maixcam_frame_count = 0U;
volatile uint32_t maixcam_last_update_ms = 0U;
uint8_t WuLiaoData=0,move_maix=3;

uint8_t mode1[3]={0xFF,0x01,0xFE},mode2[3]={0xFF,0x02,0xFE},mode3[3]={0xFF,0x03,0xFE},mode4[3]={0xFF,0x04,0xFE},\
mode5[3]={0xFF,0x05,0xFE},mode6[3]={0xFF,0x06,0xFE},mode7[3]={0xFF,0x07,0xFE},mode8[3]={0xFF,0x08,0xFE};

uint16_t yuanhuanx_last=0,yuanhuany_last=0;
void Maixcam_setmode(uint8_t mode)
{
	if(mode==1)              //物料模式
	{
		UART_SendbyteOne(&huart1,mode1,3);
		HAL_Delay(3);		
	}
	else if(mode==2)         //地面圆环定位
	{
		UART_SendbyteOne(&huart1,mode2,3);
		HAL_Delay(3);	
	}
	else if(mode==3)         //只识别红色物料
	{
		UART_SendbyteOne(&huart1,mode3,3);
		HAL_Delay(3);	
	}
	else if(mode==4)        //只识别绿色物料
	{
		UART_SendbyteOne(&huart1,mode4,3);
		HAL_Delay(3);		
	}
	else if(mode==5)        
	{
		UART_SendbyteOne(&huart1,mode5,3);
		HAL_Delay(3);		
	}
	else if(mode==6)        
	{
		UART_SendbyteOne(&huart1,mode6,3);
		HAL_Delay(3);		
	}
	else if(mode==7)        
	{
		UART_SendbyteOne(&huart1,mode7,3);
		HAL_Delay(3);		
	}
	else if(mode==8)        
	{
		UART_SendbyteOne(&huart1,mode8,3); /* FF 08 FE，禁止越界读第4个字节。 */
		HAL_Delay(3);		
	}
}
/******************************************
函数功能：对Maixcam接收的协议包进行解析
入口参数：原始协议包
返回  值：0：解析错误，协议包不正确
					1：解析正确，是要接收的数据
*******************************************/
static int maixcam_data_parse(uint8_t *pack)
{
    if (pack[0] != 0xB3)                     return 0;
    if (pack[1] != 0xB3)                     return 0;
    if (pack[maixcam_DATA_SIZE - 1] != 0x5B) return 0;
    return 1;
}

void USART1_IRQHandler_uaer(void)
{
    static uint8_t i = 0;
    static uint8_t state = 0;   /* 0=等第一个B3  1=等第二个B3  2=收数据 */
    static uint8_t maixcam_data_buff[maixcam_DATA_SIZE] = {0};
    uint8_t rx = (uint8_t)RXbuf1;

    switch (state)
    {
    case 0:                                     /* 等第一个帧头 0xB3 */
        if (rx == 0xB3) state = 1;
        break;

    case 1:                                     /* 等第二个帧头 0xB3 */
        if (rx == 0xB3)
        {
            maixcam_data_buff[0] = 0xB3;
            maixcam_data_buff[1] = 0xB3;
            i = 2;
            state = 2;
        }
        else
        {
            state = 0;                          /* 第二个不是 B3，重新找帧头 */
        }
        break;

    case 2:                                     /* 收剩余 8 字节 */
        maixcam_data_buff[i++] = rx;
        if (i >= maixcam_DATA_SIZE)             /* 10 字节收满 */
        {
            state = 0;
            if (maixcam_data_parse(maixcam_data_buff))
            {
                uint8_t  cmd   = maixcam_data_buff[2];
                uint16_t x     = ((uint16_t)maixcam_data_buff[3] << 8) | maixcam_data_buff[4];
                uint16_t y     = ((uint16_t)maixcam_data_buff[5] << 8) | maixcam_data_buff[6];
                WuLiaoData = maixcam_data_buff[8];   /* 物料=颜色码，圆环=0 */

                YuanData[0] = x;
                YuanData[1] = y;
                move_maix   = cmd;                       /* 0xB1 物料 / 0xB2 圆环 */
                maixcam_last_update_ms = HAL_GetTick();
                maixcam_frame_count++;

                /* pack[7]的半径/圆环编号目前未参与业务，保留原协议布局。
                 * 去掉未使用的局部副本，不影响坐标、颜色或帧计数更新。
                 */

                yuanhuanx_last = YuanData[0];
                yuanhuany_last = YuanData[1];
            }
        }
        break;
    }
}
