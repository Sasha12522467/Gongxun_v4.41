#include "actionInit.h"
#include "usart.h"
#include "tim.h"
#include "FSUS.h"
#include "Emm_V5.h"
#include "cmsis_os.h"
#include "maixcam.h"

void action_Init(void)
 {	 
  __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1700);  //夹紧950   完全打开1700    
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,700);//tim3 ch2 1550初始化  
  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1500);  
	
  ser_set(-145,1000);   
  ZDT_set(6,1300);
	HAL_Delay(500);
	ZDT_set(1,0);
	 Maixcam_setmode(1U);
	HAL_Delay(300);
	 

 }

void ZDT_set(uint8_t id , uint16_t num)
{
	if(id==1)
	{
		Emm_V5_Pos_Control(1,1, 230,240,num, true, false); 
	}
 	if(id==6)
	{
	 Emm_V5_Pos_Control(6,0, 230,240,num, true, false); 
	}
}
 
 
/* Faster motor profile used only by coarse-processing retrieval. */
void ZDT_set2(uint8_t id, uint16_t num)
{
    if (id == 1U)
    {
        Emm_V5_Pos_Control(1,1,550,240,num,true,false);
    }
    if (id == 6U)
    {
        Emm_V5_Pos_Control(6,0,550,240,num,true,false);
    }
}

void servo_set(uint16_t num)  
{
		if(num==1)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1400);  
			osDelay(500);
		}
		if(num==2)
		{
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1700);
			osDelay(500);
		}
}
 
void Plate(uint16_t num)
{
		if(num==1)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1750);  //500转盘1 1100转盘2 1600转盘3

		}
		if(num==2)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1250);

		}
		if(num==3)
		{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,650);
		}

}
