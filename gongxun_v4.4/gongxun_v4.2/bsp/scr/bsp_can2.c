#include "bsp_can2.h"
#include "can.h"
#include "main.h"

CAN_TxHeaderTypeDef canTxHeader = {0};
CAN_RxHeaderTypeDef canRxHeader = {0};
uint8_t canTxData[8] = {0};
uint8_t canRxData[8] = {0};
uint32_t txMailBox = 0U;

volatile uint8_t can_init_step = 0U;
volatile HAL_StatusTypeDef can_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef can_tx_status = HAL_ERROR;
volatile HAL_StatusTypeDef can_rx_status = HAL_ERROR;
volatile uint32_t can_tx_count = 0U;
volatile uint32_t can_rx_count = 0U;
volatile uint32_t dm_feedback_count = 0U;
volatile uint8_t dm_state = 0xFFU;
volatile uint8_t dm_motor_id = 0xFFU;
volatile uint32_t dm_last_feedback_ms = 0U;

void canfilterconfig(void)
{
    CAN_FilterTypeDef sFilterConfig = {0};

    can_init_step = 0U;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000U;
    sFilterConfig.FilterIdLow = 0x0000U;
    sFilterConfig.FilterMaskIdHigh = 0x0000U;
    sFilterConfig.FilterMaskIdLow = 0x0000U;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    /* CAN1/CAN2 共用滤波器；14~27 分配给 CAN2，不能沿用 CAN1 的 0。 */
    sFilterConfig.FilterBank = 14U;
    sFilterConfig.SlaveStartFilterBank = 14U;

    can_init_status = HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
    if (can_init_status != HAL_OK)
    {
        Error_Handler();
    }
    can_init_step = 1U;

    can_init_status = HAL_CAN_Start(&hcan2);
    if (can_init_status != HAL_OK)
    {
        Error_Handler();
    }
    can_init_step = 2U;

    can_init_status = HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (can_init_status != HAL_OK)
    {
        Error_Handler();
    }
    can_init_step = 3U;
}

void can_tx(float _pos, float _vel)
{
    uint8_t *pbuf = (uint8_t *)&_pos;
    uint8_t *vbuf = (uint8_t *)&_vel;

    canTxHeader.StdId = 0x100U + DM_CAN_ID;
    canTxHeader.ExtId = 0x00U;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.IDE = CAN_ID_STD;
    canTxHeader.DLC = 8U;
    canTxHeader.TransmitGlobalTime = DISABLE;

    canTxData[0] = pbuf[0];
    canTxData[1] = pbuf[1];
    canTxData[2] = pbuf[2];
    canTxData[3] = pbuf[3];
    canTxData[4] = vbuf[0];
    canTxData[5] = vbuf[1];
    canTxData[6] = vbuf[2];
    canTxData[7] = vbuf[3];

    can_tx_count++;
    can_tx_status = HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
    if (can_tx_status != HAL_OK)
    {
        Error_Handler();
    }
}

void DM_Enable(void)
{
    canTxHeader.StdId = 0x100U + DM_CAN_ID;
    canTxHeader.ExtId = 0x00U;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.IDE = CAN_ID_STD;
    canTxHeader.DLC = 8U;
    canTxHeader.TransmitGlobalTime = DISABLE;

    canTxData[0] = 0xFFU;
    canTxData[1] = 0xFFU;
    canTxData[2] = 0xFFU;
    canTxData[3] = 0xFFU;
    canTxData[4] = 0xFFU;
    canTxData[5] = 0xFFU;
    canTxData[6] = 0xFFU;
    canTxData[7] = 0xFCU;

    can_tx_count++;
    can_tx_status = HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &txMailBox);
    if (can_tx_status != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan2)
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        can_rx_status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canRxHeader, canRxData);
        if (can_rx_status != HAL_OK)
        {
            break;
        }
        can_rx_count++;

        /* 只将当前电机的标准数据反馈帧解析为状态，其他帧仍保留在全局收包变量中。 */
        if (canRxHeader.IDE == CAN_ID_STD && canRxHeader.RTR == CAN_RTR_DATA &&
            canRxHeader.StdId == DM_MASTER_ID && canRxHeader.DLC == 8U &&
            (canRxData[0] & 0x0FU) == (DM_CAN_ID & 0x0FU))
        {
            dm_motor_id = canRxData[0] & 0x0FU;
            dm_state = (canRxData[0] >> 4) & 0x0FU;
            dm_last_feedback_ms = HAL_GetTick();
            dm_feedback_count++;
        }
    }
}
