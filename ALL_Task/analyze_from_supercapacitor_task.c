#include "analyze_from_supercapacitor_task.h"
#include "cmsis_os.h"
#include "../Application/robot_global.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"

// 定义 CAN 接收消息的结构体（需与 bsp_can.c 中保持一致）
typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
} can_rx_msg_t;

// 实例化队列句柄，供全局（如 bsp_can.c）调用
QueueHandle_t supercap_can_rx_queue;

void analyze_from_supercapacitor_task_func(void const * argument)
{
    // 创建消息队列，深度为 10，足够缓冲突发帧
    supercap_can_rx_queue = xQueueCreate(10, sizeof(can_rx_msg_t));
    can_rx_msg_t rx_msg;

    for(;;)
    {
        // 阻塞等待队列中的 CAN 数据，portMAX_DELAY 意味着没有数据时此任务完全不占用 CPU
        if (xQueueReceive(supercap_can_rx_queue, &rx_msg, portMAX_DELAY) == pdTRUE)
        {
            /* 解析 0x301 数据包 
             * 默认两个 STM32 通讯为小端模式（低字节在前，高字节在后）
             * 按位移拼接比强制指针转换更安全，能避免结构体内存对齐引发的 Bug
             */
             
            // 1. 电容剩余电压
            robot_ctrl.supercap.capacity_voltage = (int16_t)(rx_msg.data[1] << 8 | rx_msg.data[0]);
            
            // 2. 底盘实时输出功率
            robot_ctrl.supercap.chassis_output_power = (int16_t)(rx_msg.data[3] << 8 | rx_msg.data[2]);
            
            // 3. 电容实时充电功率
            robot_ctrl.supercap.cap_charge_power = (int16_t)(rx_msg.data[5] << 8 | rx_msg.data[4]);
            
            // 4. 温度
            robot_ctrl.supercap.temperature = rx_msg.data[6];
            
            // 5. 状态标志位
            robot_ctrl.supercap.status = rx_msg.data[7];
        }
    }
}