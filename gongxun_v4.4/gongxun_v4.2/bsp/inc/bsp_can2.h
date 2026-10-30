#ifndef __BSP_CAN2_H__
#define __BSP_CAN2_H__

#include "stm32f4xx_hal.h"

/* 与当前发送 ID 0x101 对应；Master ID 按电机调试助手中的设置填写。 */
#define DM_CAN_ID       0x01U
#define DM_MASTER_ID    0x00U

/* 与参考工程一样，将收发帧保存成全局变量，供 Debug / Watch 查看。 */
extern CAN_TxHeaderTypeDef canTxHeader;
extern CAN_RxHeaderTypeDef canRxHeader;
extern uint8_t canTxData[8];
extern uint8_t canRxData[8];
extern uint32_t txMailBox;

extern volatile uint8_t can_init_step;      /* 0:未开始，1:滤波完成，2:启动完成，3:通知开启。 */
extern volatile HAL_StatusTypeDef can_init_status;
extern volatile HAL_StatusTypeDef can_tx_status; /* 仅表示提交发送邮箱的结果。 */
extern volatile HAL_StatusTypeDef can_rx_status;
extern volatile uint32_t can_tx_count;      /* 提交发送的次数，包含失败尝试。 */
extern volatile uint32_t can_rx_count;      /* CAN2 所有成功收到的帧数。 */
extern volatile uint32_t dm_feedback_count; /* 匹配 Master ID 和电机 ID 的反馈帧数。 */
extern volatile uint8_t dm_state;           /* 0xFF:未收到反馈；0:失能；1:使能；9:欠压。 */
extern volatile uint8_t dm_motor_id;
extern volatile uint32_t dm_last_feedback_ms;

/* MX_CAN2_Init() 后依次调用 canfilterconfig()、DM_Enable()，各一次。 */
void canfilterconfig(void);
void DM_Enable(void);

/* 与参考工程一致：位置和速度两个 float，单位为 rad 和 rad/s。 */
void can_tx(float _pos, float _vel);

#endif
