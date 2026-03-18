#include "bsp_can.h"
#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} can_rx_msg_t;

extern QueueHandle_t supercap_can_rx_queue;

/**
 * @brief CAN FIFO0 接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_rx_msg_t rx_msg;

    // 处理 CAN1 的接收
    if (hcan->Instance == CAN1) {
        // 1. 读取硬件接收 FIFO 中的数据
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_msg.header, rx_msg.data);

        // 2. 根据电容头文件，电容板发送ID为 0x301 (TxID)，所以主控接收ID为 0x301
        if (rx_msg.header.StdId == 0x301) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            // 3. 将数据压入队列，唤醒解析 Task
            if (supercap_can_rx_queue != NULL) {
                xQueueSendFromISR(supercap_can_rx_queue, &rx_msg, &xHigherPriorityTaskWoken);
                // 如果唤醒的 Task 优先级比当前中断打断的 Task 高，则立刻触发任务调度
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }

    }
}

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
