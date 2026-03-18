#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 配置 CAN 过滤器及启动中断
 */
void bsp_can_init(void)
{
    CAN_FilterTypeDef can_filter_st;

    // 配置通用参数
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    // --- 配置 CAN1 ---
    can_filter_st.FilterBank = 0; // CAN1 使用 0 号过滤器
    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // --- 配置 CAN2 ---
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14; // CAN2 使用 14 号过滤器
    if (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK) {
        Error_Handler();
    }
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
