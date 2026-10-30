#include "robot_task.h"
#include "chassic.h"
#include "tim.h"
#include "action.h"
#include "actionInit.h"
#include "cmsis_os.h"

uint8_t material_last = 0U; /* 0: return to middle; 1: last raw grab. */

void grab_middle(int num)
{
	ZDT_set(6,1300);
osDelay(400);
    ZDT_set(1,4400);//下降
    osDelay(500);
    Plate(num);
	servo_set(1);//抓紧
    ZDT_set(1,1500);//上升速度更快
    osDelay(200);
    ZDT_set(6,400);
    osDelay(100);
    ser_set(-145,750);
    osDelay(400);
    ZDT_set(1,2600);
    osDelay(400);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(200);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
void grab_left(int num)
{	
ser_set(-81,400);
osDelay(200);
ZDT_set(6,9800);
osDelay(300);
    ZDT_set(1,4400);//下降
    osDelay(500);
Plate(num);
servo_set(1);
    ZDT_set(1,1500);//上升速度更快
	osDelay(300);
    ZDT_set(6,400);
    osDelay(400);
    ser_set(-145,700);
	osDelay(300);
	ZDT_set(1,2600);
    osDelay(400);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(100);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
void grab_right(int num)
{
ser_set(-54,400);
osDelay(200);
ZDT_set(6,9800);
osDelay(300);
    ZDT_set(1,4400);//下降
    osDelay(500);
Plate(num);
servo_set(1);
    ZDT_set(1,0);
	osDelay(600);
    ZDT_set(6,400);
    osDelay(300);
    ser_set(-145,900);
	osDelay(400);
	ZDT_set(1,2600);
    osDelay(500);
    servo_set(2); 
    ZDT_set(1,0);
    osDelay(100);
    if (material_last == 0U)
    {
        ZDT_set(6,5500);
        osDelay(300);
        ser_set(-69,800);
        osDelay(810);
    }
}
