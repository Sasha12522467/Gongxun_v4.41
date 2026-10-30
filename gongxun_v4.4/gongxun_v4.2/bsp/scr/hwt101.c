#include "hwt101.h"
#include "bsp_usart.h"

int hwt101_data[2]={0};
volatile float yaw=0;    //陀螺仪的角度
static volatile uint32_t s_hwt101_frame_count=0;
static volatile uint32_t s_hwt101_last_update_ms=0;

float kp=-2.3f,kd=-0.1f;
float my_abs_1(float num) 
{
    return (num < 0) ? -num : num;
}

void Limit(float Soruce,float Limit_Max,float Limit_Min)
{
	if(Soruce>Limit_Max)   Soruce=Limit_Max;
	if(Soruce<Limit_Min)   Soruce=Limit_Min;
}

float imu_pid(float hope)
{
	static float err;
	static float err_last_dev;
	static float err_last;
	float out;
	
//	(*para).err = yaw - hope;
     err=yaw - hope;
     /* 原料定位目标已转换为IMU原始航向；±180度跳变取最短角差，
      * 不改变零点。此处直接使用本次yaw计算，避免外部快照过时造成绕大圈。
      */
     while(err>180.0f) err-=360.0f;
     while(err<=-180.0f) err+=360.0f;
//	(*para).err_last_dev = (*para).err - (*para).err_last;
	    err_last_dev=err-err_last;
//	(*para).out = (*para).kp*(*para).err + (*para).kd*(*para).err_last_dev;
	     out=kp*err+kd*err_last_dev;
//	(*para).err_last =  (*para).err;
				err_last=err;
//	(*para).err_add+=(*para).err;
	if(my_abs_1(err)>50&&my_abs_1(err)<100)
	{
		Limit(out,210,-210);
	}
	else if(my_abs_1(err)>15&&my_abs_1(err)<=50)
	{
		Limit(out,70,-70);
	}
	else if(my_abs_1(err)>=0&&my_abs_1(err)<=15)
	{
		Limit(out,40,-40);
	}	
	
	
	if(out<1&&out>0)
	{
		out=1;
	}
	else if(out>-1&&out<0)
	{
		out=-1;
	}
	
//	if((*para).out<0)(*para).out=-(*para).out;
	
	return out;
}


/**
  * @brief HWT101的数据验证
  * @param 
  * @retval 无
  */
void hwt101_Get(void)
{
    int16_t raw = (int16_t)((hwt101_data[1] << 8) | hwt101_data[0]);
    yaw = (float)raw / 32768.0f * 180.0f;
}

static uint8_t HWT101_ChecksumOk(const uint8_t *frame)
{
	uint8_t sum=0;
	uint8_t i;
	for(i=0;i<10U;i++) sum=(uint8_t)(sum+frame[i]);
	return (sum==frame[10]) ? 1U : 0U;
}

void USART6_IRQHandler_uaer(uint16_t size)
{
	uint16_t i;
	HAL_UART_DMAStop(&huart6);
	if(size>HWT101_RX_BUFFER_SIZE) size=HWT101_RX_BUFFER_SIZE;

	/* Search the DMA block for a complete 0x55 0x53 angle frame. */
	for(i=0U;(i+10U)<size;i++)
	{
		if(RXbuf6[i]==0x55U && RXbuf6[i+1U]==0x53U &&
		   HWT101_ChecksumOk(&RXbuf6[i]))
		{
			int16_t raw=(int16_t)(((uint16_t)RXbuf6[i+7U]<<8) |
			                       RXbuf6[i+6U]);
			hwt101_data[0]=RXbuf6[i+6U];
			hwt101_data[1]=RXbuf6[i+7U];
			yaw=(float)raw/32768.0f*180.0f;
			s_hwt101_last_update_ms=HAL_GetTick();
			s_hwt101_frame_count++;
			i+=10U;
		}
	}
	HWT101_ResReceiveToIdle_DMA();
}

void HWT101_GetSnapshot(HWT101_Snapshot_t *snapshot)
{
	uint32_t primask;
	if(snapshot==0) return;
	primask=__get_PRIMASK();
	__disable_irq();
	snapshot->yaw_deg=yaw;
	snapshot->frame_count=s_hwt101_frame_count;
	snapshot->last_update_ms=s_hwt101_last_update_ms;
	if(primask==0U) __enable_irq();
}

uint8_t HWT101_IsOnline(uint32_t timeout_ms)
{
	HWT101_Snapshot_t snapshot;
	HWT101_GetSnapshot(&snapshot);
	if(snapshot.frame_count==0U) return 0U;
	return ((uint32_t)(HAL_GetTick()-snapshot.last_update_ms)<=timeout_ms)
	       ? 1U : 0U;
}


uint8_t A101_1[]={0xFF,0xAA,0x76,0x00,0x00};  //Z轴角度归零
uint8_t A101_2[]={0xFF,0xAA,0x48,0x01,0x00};   
uint8_t A101_3[]={0xFF,0xAA,0x48,0x00,0x00};
void Hwt101_Yaw_Res(void)
{
	UART_SendbyteOne(&huart6,A101_1,5);
	HAL_Delay(500);
}
void hwt101_JiaoZun(void)
{
	uint8_t i=0;
  UART_SendbyteOne(&huart6,A101_2,5);
	for(i=0;i<22;i++) HAL_Delay(1000);
	UART_SendbyteOne(&huart6,A101_3,5);
}
