#include "FSUS.h"
#include "bsp_can2.h"
//-70  -7  13
Usart_DataTypeDef *servo_usart = &FSUS_Usart; // 串口总线舵机对应的USART
void ser_set(float angle_set,uint16_t time)
{
	// 舵机的ID号
	uint8_t servo_id = 1;
	// 舵机的目标角度
	// 舵机角度在-180度到180度之间, 最小单位0.1
	float angle = 0;
	// 时间间隔ms
	// 可以尝试修改设置更小的时间间隔，例如500ms
	uint16_t interval;
	// 加速时间
	uint16_t t_acc;
	// 减速时间
	uint16_t t_dec;
	// 舵机执行功率 mV 默认为0
	uint16_t power = 0;

	// 控制舵机角度 + 指定时间
	angle =angle_set;
	interval = time;
	t_acc = 100;
	t_dec = 150;
	FSUS_SetServoAngleByInterval(servo_usart, servo_id, angle, interval, t_acc, t_dec, power);

}

/* CAN motor: position and speed use the reference project's native units. */
void ser_set2(float angle_set, float speed)
{
    can_tx(angle_set, speed);
}
