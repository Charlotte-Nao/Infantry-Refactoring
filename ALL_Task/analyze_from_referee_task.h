//
// Created by ASUS on 2026/3/19.
//

#ifndef INFANTRY_01_ANALYZE_FROM_REFEREE_TASK_H
#define INFANTRY_01_ANALYZE_FROM_REFEREE_TASK_H

#include "stdint.h"

#define REF_RX_BUF_SIZE 256
#define REF_HEADER_SOF  0xA5

// 1. 官方协议帧头结构体 (5字节)
typedef struct __attribute__((packed)) {
    uint8_t  SOF;          // 起始字节 0xA5
    uint16_t data_length;  // 数据帧中 data 的长度
    uint8_t  seq;          // 包序号
    uint8_t  CRC8;         // 帧头 CRC8 校验
} frame_header_struct_t;

// 比赛状态数据：0x0001 (11字节)
typedef struct __attribute__((packed)) {
    uint8_t game_type : 4;       // 比赛类型: 1:RMUC, 2:RMUT, 3:RMUL, 4:3V3, 5:1V1
    uint8_t game_progress : 4;   // 比赛阶段: 0:未开始, 1:准备区, 2:自检区, 3:5秒倒计时, 4:比赛中, 5:结算中
    uint16_t stage_remain_time;  // 当前阶段剩余时间 (单位：秒)
    uint64_t SyncTimeStamp;      // 机器人与裁判系统时间同步的 UNIX 时间戳 (微秒)
} ext_game_status_t;

// 0x0101场地信息（4字节）
typedef struct __attribute__((packed)) {
    uint32_t place_t;           //场地信息
} ext_place_status_t;

// 0x0201 机器人性能状态数据 (13字节) —— 【云台/底盘核心】
typedef struct __attribute__((packed)) {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_game_robot_status_t;

// 0x0202 实时功率热量数据 (14字节) —— 【拨弹轮防超热量核心】
typedef struct __attribute__((packed)) {
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} ext_power_heat_data_t;

// 0x0203 机器人绝对位置数据 (12字节) —— 【哨兵自主导航核心】
typedef struct __attribute__((packed)) {
    float x;
    float y;
    float yaw;
} ext_game_robot_pos_t;

// 0x0206 机器人受击数据（受击情况）
typedef struct __attribute__((packed)) {
    uint8_t armor_id : 4;
    uint8_t HP_deducation_reason : 4;
} ext_huart_robot_data_t;

// 0x0208 机器人发弹相关（弹药情况）
typedef struct __attribute__((packed)) {
    uint16_t allow_bullet_17;
    uint16_t allow_bullet_42;
    uint16_t money_left;
    uint16_t extra_bullet;
} ext_allow_robot_data_t;

// 裁判系统总控结构体
typedef struct {
    ext_game_status_t       game_status;     // 包含：比赛阶段、剩余时间
    ext_place_status_t      place_status;    // 包含： 场地信息
    ext_game_robot_status_t robot_status;    // 包含：等级、血量、热量上限、功率上限
    ext_power_heat_data_t   power_heat_data; // 包含：当前17mm热量、缓冲能量
    ext_game_robot_pos_t    robot_pos;       // 包含：X, Y, Z, Yaw 坐标与朝向
    ext_huart_robot_data_t  huart_robot;     // 包含受伤情况
    ext_allow_robot_data_t  allow_robot;     // 弹药可用情况

    uint32_t last_update_tick; // 掉线检测时间戳
} referee_info_t;


// 接口与任务声明
void analyze_from_referee_task_func(void const * argument);
void Referee_Init(void);
void Referee_Data_Parse(uint8_t *rx_buf, uint16_t len);
void Referee_Send_Packet(uint16_t cmd_id, uint8_t *data, uint16_t data_len);


#endif //INFANTRY_01_ANALYZE_FROM_REFEREE_TASK_H