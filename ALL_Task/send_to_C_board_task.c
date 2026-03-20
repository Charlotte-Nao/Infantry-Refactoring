//
// Created by ASUS on 2026/3/19.
//

#include "send_to_C_board_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "../Application/robot_global.h"

extern CAN_HandleTypeDef hcan2; // 声明外部 CAN2 句柄

/**
  * @brief  数据更新及 CAN2 发送函数
  */
static void Send_To_C_Board_CAN_Forward(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t send_mail_box;

    // ========================================================
    // 1. 数据刷新：从裁判系统和超级电容中提取最新数据到网关结构体
    // ========================================================
    robot_ctrl.gateway_c_board.buffer_energy = robot_ctrl.referee_info.power_heat_data.buffer_energy;
    robot_ctrl.gateway_c_board.shooter_17mm_barrel_heat = robot_ctrl.referee_info.power_heat_data.shooter_17mm_barrel_heat;

    robot_ctrl.gateway_c_board.capacity_voltage = robot_ctrl.supercap.capacity_voltage;
    robot_ctrl.gateway_c_board.chassis_output_power = robot_ctrl.supercap.chassis_output_power;

    robot_ctrl.gateway_c_board.robot_id = robot_ctrl.referee_info.robot_status.robot_id;
    robot_ctrl.gateway_c_board.HP_deducation_reason = robot_ctrl.referee_info.huart_robot.HP_deducation_reason;


    // ========================================================
    // 2. 发送第一帧 (ID: 0x101) - 核心高频数据 (8字节满载)
    // ========================================================
    tx_header.StdId = 0x101;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    // 大端序拆解 (高8位在前，低8位在后)
    tx_data[0] = (uint8_t)(robot_ctrl.gateway_c_board.buffer_energy >> 8);
    tx_data[1] = (uint8_t)(robot_ctrl.gateway_c_board.buffer_energy & 0xFF);

    tx_data[2] = (uint8_t)(robot_ctrl.gateway_c_board.shooter_17mm_barrel_heat >> 8);
    tx_data[3] = (uint8_t)(robot_ctrl.gateway_c_board.shooter_17mm_barrel_heat & 0xFF);

    tx_data[4] = (uint8_t)(robot_ctrl.gateway_c_board.capacity_voltage >> 8);
    tx_data[5] = (uint8_t)(robot_ctrl.gateway_c_board.capacity_voltage & 0xFF);

    tx_data[6] = (uint8_t)(robot_ctrl.gateway_c_board.chassis_output_power >> 8);
    tx_data[7] = (uint8_t)(robot_ctrl.gateway_c_board.chassis_output_power & 0xFF);

    // 检查邮箱是否满，避免阻塞死锁
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
    }

    // ========================================================
    // 3. 发送第二帧 (ID: 0x102) - 附加状态数据 (2字节)
    // ========================================================
    tx_header.StdId = 0x102;
    tx_header.DLC = 2; // 只包含两个变量，设置 DLC 为 2 节省总线带宽

    tx_data[0] = robot_ctrl.gateway_c_board.robot_id;
    tx_data[1] = robot_ctrl.gateway_c_board.HP_deducation_reason;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, &send_mail_box);
    }
}

/**
  * @brief  发送至 C 板的 FreeRTOS 任务
  */
void send_to_C_board_task_func(void const * argument)
{

    for(;;)
    {
        // 核心打包发送逻辑
        Send_To_C_Board_CAN_Forward();

        // 延时 10ms，即发送频率为 100Hz
        // 足以保证多板协同的实时性，又能避免过度占用 CAN 总线导致塞车
        osDelay(10);
    }
}