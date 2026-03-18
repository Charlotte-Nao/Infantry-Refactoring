//
// Created by ASUS on 2026/3/19.
//

#include "cmsis_os.h"
#include "main.h"
#include "../Application/robot_global.h"

// 引入你封装的 UART BSP 头文件
// 注意：如果编译提示找不到文件，请根据你的工程实际相对路径修改
#include "../Bsp/uart/bsp_uart.h"

/**
  * @brief  发送至裁判系统 / 串口调试打印的 Task
  */
void send_to_referee_task_func(void const * argument)
{
    // 消除编译器针对未使用参数的警告
    (void)argument;

    // 1. 获取封装好的 UART1_DMA 设备指针
    struct uart_device *uart1 = uart_get_device("uart1_dma");

    // 2. 初始化该设备 (极其重要：这里会创建 xTxSem 信号量和队列)
    if (uart1 != NULL && uart1->Init != NULL) {
        // 参数意义: baud=115200, datas=8, parity='N', stop=1 (底层 Init 虽然没用上，但按规范传参)
        uart1->Init(uart1, 115200, 8, 'N', 1);
    }

    for(;;)
    {
        // 3. 检查设备是否成功获取，并周期性调用其内部的 Print 函数
        if (uart1 != NULL && uart1->Print != NULL) {

            // 打印我们刚刚在 send_to_C_board_task 中打包好的网关数据
            uart1->Print(uart1,
                "========== GATEWAY TO C-BOARD ==========\r\n"
                " [Robot ID] : %d\r\n"
                " [Buffer]   : %d J\r\n"
                " [Heat 17]  : %d\r\n"
                " [Cap Volt] : %d (x100 V)\r\n"
                " [Chassis P]: %d (x10 W)\r\n"
                " [Hurt Rsn] : %d\r\n"
                "========================================\r\n\r\n",
                robot_ctrl.gateway_c_board.robot_id,
                robot_ctrl.gateway_c_board.buffer_energy,
                robot_ctrl.gateway_c_board.shooter_17mm_barrel_heat,
                robot_ctrl.gateway_c_board.capacity_voltage,
                robot_ctrl.gateway_c_board.chassis_output_power,
                robot_ctrl.gateway_c_board.HP_deducation_reason
            );
        }

        // 延时 500 毫秒，即 2Hz 打印频率。
        // 由于这里调用的是阻塞/等待信号量的 printf，太快可能会卡死或挤爆缓冲区
        osDelay(500);
    }
}