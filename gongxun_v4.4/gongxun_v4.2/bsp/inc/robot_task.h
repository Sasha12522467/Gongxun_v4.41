#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "main.h"  // 包含HAL库相关及UART句柄定义
extern uint8_t material_last;
void grab_middle(int num);
void grab_left(int num);
void grab_right(int num);


//后续加工区域
void task_Rawmaterial(void);
void task_Processing(void);
void task_Storage(void);
#endif /* __USART3_H */
