//
// Created by ASUS on 2026/3/19.
//

#include "send_to_referee_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"

// 引入系统全局变量和裁判系统底层接口
#include "../Application/robot_global.h"

/* ==================== 官方 UI 协议宏定义 ==================== */
#define UI_SEND_PERIOD_MS            100U
#define UI_HIT_HIGHLIGHT_MS          300U
#define UI_CAP_VOLTAGE_MIN_X100      0
#define UI_CAP_VOLTAGE_MAX_X100      3000
#define UI_POWER_MIN_X10             0
#define UI_POWER_MAX_X10             2000

// 左上角状态区布局
#define UI_STATUS_BAR_LEFT           50U
#define UI_STATUS_BAR_RIGHT          520U
#define UI_CAP_BAR_Y                 860U
#define UI_PWR_BAR_Y                 800U

// 左侧装甲受击示意布局
#define UI_CAR_X0                    220U
#define UI_CAR_Y0                    600U
#define UI_CAR_X1                    340U
#define UI_CAR_Y1                    720U

// UI 操作与颜色宏
#define REF_UI_OP_ADD      1
#define REF_UI_OP_MODIFY   2
#define REF_UI_OP_DELETE   3

#define REF_UI_TYPE_LINE   0
#define REF_UI_TYPE_RECT   1

#define REF_UI_COLOR_RED_BLUE 0
#define REF_UI_COLOR_YELLOW   1
#define REF_UI_COLOR_GREEN    2
#define REF_UI_COLOR_ORANGE   3
#define REF_UI_COLOR_WHITE    8

/* ==================== 官方 UI 协议结构体 ==================== */

// 图形数据结构体 (15字节，严格对齐)
typedef struct __attribute__((packed)) {
    uint8_t figure_name[3];
    uint32_t operate_type:3;
    uint32_t figure_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

// 交互数据帧头格式 (0x0301)
typedef struct __attribute__((packed)) {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
} ext_interaction_header_t;

// 删除图层包 (0x0100)
typedef struct __attribute__((packed)) {
    ext_interaction_header_t header;
    uint8_t delete_type;
    uint8_t layer;
} ext_ui_delete_t;

// 绘制5个图形包 (0x0103)
typedef struct __attribute__((packed)) {
    ext_interaction_header_t header;
    interaction_figure_t figures[5];
} ext_ui_draw5_t;


/* ==================== 内部工具函数 ==================== */

static uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

static uint8_t armor_color_by_tick(uint32_t now, uint32_t last_hit_tick) {
    return ((now - last_hit_tick) <= UI_HIT_HIGHLIGHT_MS) ? REF_UI_COLOR_ORANGE : REF_UI_COLOR_GREEN;
}

static int8_t armor_id_to_index(uint8_t armor_id) {
    if (armor_id <= 3U) return (int8_t)armor_id;
    if (armor_id == 4U) return 3;
    return -1;
}

// 封装发送：删除图层
static void UI_Delete_All(uint16_t sender_id, uint16_t receiver_id) {
    ext_ui_delete_t pkt;
    pkt.header.data_cmd_id = 0x0100;
    pkt.header.sender_id = sender_id;
    pkt.header.receiver_id = receiver_id;
    pkt.delete_type = 2; // 2:删除所有
    pkt.layer = 0;

    // CMD_ID 0x0301 (交互数据)，发送总长度为头部(6) + 内容(2) = 8字节
    Referee_Send_Packet(0x0301, (uint8_t*)&pkt, sizeof(pkt));
}

// 封装发送：绘制5个图形
static void UI_Draw_5(uint16_t sender_id, uint16_t receiver_id, interaction_figure_t* figs) {
    ext_ui_draw5_t pkt;
    pkt.header.data_cmd_id = 0x0103;
    pkt.header.sender_id = sender_id;
    pkt.header.receiver_id = receiver_id;
    memcpy(pkt.figures, figs, sizeof(interaction_figure_t) * 5);

    // CMD_ID 0x0301，发送总长度为头部(6) + 5个图形(5*15) = 81字节
    Referee_Send_Packet(0x0301, (uint8_t*)&pkt, sizeof(pkt));
}

// 构造矩形
static void make_rect(interaction_figure_t *fig, const char name0, const char name1, const char name2,
                      uint8_t op, uint8_t color, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t width) {
    memset(fig, 0, sizeof(*fig));
    fig->figure_name[0] = (uint8_t)name0;
    fig->figure_name[1] = (uint8_t)name1;
    fig->figure_name[2] = (uint8_t)name2;
    fig->operate_type = op;
    fig->figure_type = REF_UI_TYPE_RECT;
    fig->layer = 0U;
    fig->color = color;
    fig->width = width;
    fig->start_x = x0;
    fig->start_y = y0;
    fig->details_d = x1;
    fig->details_e = y1;
}

// 构造线条
static void make_line(interaction_figure_t *fig, const char name0, const char name1, const char name2,
                      uint8_t op, uint8_t color, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t width) {
    memset(fig, 0, sizeof(*fig));
    fig->figure_name[0] = (uint8_t)name0;
    fig->figure_name[1] = (uint8_t)name1;
    fig->figure_name[2] = (uint8_t)name2;
    fig->operate_type = op;
    fig->figure_type = REF_UI_TYPE_LINE;
    fig->layer = 0U;
    fig->color = color;
    fig->width = width;
    fig->start_x = x0;
    fig->start_y = y0;
    fig->details_d = x1;
    fig->details_e = y1;
}


/* ==================== 核心任务 ==================== */

void send_to_referee_task_func(void const * argument) {
    (void)argument;

    uint8_t cleared_once = 0U;
    uint8_t status_drawn_once = 0U;
    uint8_t armor_drawn_once = 0U;
    uint8_t send_selector = 0U;
    uint32_t last_send_tick = 0U;

    uint16_t last_sender_id = 0U;
    uint16_t last_receiver_id = 0U;

    uint8_t last_hurt_sig = 0U;
    uint16_t last_hp = 0xFFFFU;
    uint32_t armor_hit_tick[4] = {0U, 0U, 0U, 0U}; // 0前 1左 2后 3右

    while (1) {
        // 从全局变量拉取数据
        uint16_t sender_id = robot_ctrl.referee_info.robot_status.robot_id;
        uint16_t receiver_id = 0;

        // 如果没有接上裁判系统，ID 为 0，不发送
        if (sender_id != 0U) {
            // 根据协议计算客户端 ID (本车 ID + 0x0100)
            receiver_id = sender_id + 0x0100;

            // 当机器人 ID 变化 (比如重启/掉线重连) 时，重置标志位
            if ((sender_id != last_sender_id) || (receiver_id != last_receiver_id)) {
                cleared_once = 0U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_sender_id = sender_id;
                last_receiver_id = receiver_id;
            }

            // 1. 初次连接，清除屏幕所有残影
            if (cleared_once == 0U) {
                UI_Delete_All(sender_id, receiver_id);
                cleared_once = 1U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_send_tick = osKernelSysTick();
            }

            // 2. 10Hz (100ms) 交替发送状态栏和装甲受击指示
            if ((osKernelSysTick() - last_send_tick) >= UI_SEND_PERIOD_MS) {
                uint32_t now = osKernelSysTick();
                uint8_t op_status = (status_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;
                uint8_t op_armor = (armor_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;

                interaction_figure_t status_figs5[5];
                interaction_figure_t armor_figs5[5];

                // ---------- 计算受击高亮 ----------
                uint8_t armor_id = robot_ctrl.referee_info.huart_robot.armor_id;
                uint8_t reason = robot_ctrl.referee_info.huart_robot.HP_deducation_reason;
                int8_t armor_idx = armor_id_to_index(armor_id);
                uint8_t sig = (uint8_t)((reason << 4) | (armor_id & 0x0FU));
                uint16_t hp_now = robot_ctrl.referee_info.robot_status.current_HP;
                uint8_t hp_drop = 0U;

                if ((last_hp != 0xFFFFU) && (hp_now < last_hp)) {
                    hp_drop = 1U;
                }
                last_hp = hp_now;

                if (((sig != last_hurt_sig) || (hp_drop != 0U)) && (armor_idx >= 0)) {
                    armor_hit_tick[(uint8_t)armor_idx] = now;
                    last_hurt_sig = sig;
                }

                // ---------- 绘制超级电容进度条 ----------
                int16_t v = robot_ctrl.supercap.capacity_voltage;
                uint16_t v_clamp = clamp_u16((int32_t)v, UI_CAP_VOLTAGE_MIN_X100, UI_CAP_VOLTAGE_MAX_X100);
                uint16_t bar_left = UI_STATUS_BAR_LEFT;
                uint16_t bar_right = UI_STATUS_BAR_RIGHT;
                uint16_t bar_y = UI_CAP_BAR_Y;
                uint16_t bar_inner_left = bar_left + 8U;
                uint16_t bar_inner_right_max = bar_right - 8U;
                uint16_t inner_span = bar_inner_right_max - bar_inner_left;
                uint16_t fill_len = (uint16_t)(((uint32_t)(v_clamp - UI_CAP_VOLTAGE_MIN_X100) * inner_span) /
                                                (uint32_t)(UI_CAP_VOLTAGE_MAX_X100 - UI_CAP_VOLTAGE_MIN_X100));
                uint16_t fill_right = bar_inner_left + fill_len;

                make_rect(&status_figs5[0], 'C', 'B', '0', op_status, REF_UI_COLOR_WHITE,
                          bar_left, bar_y - 20U, bar_right, bar_y + 20U, 2U);
                make_line(&status_figs5[1], 'C', 'B', '1', op_status, REF_UI_COLOR_GREEN,
                          bar_inner_left, bar_y, fill_right, bar_y, 12U);

                // ---------- 绘制底盘功率进度条 ----------
                int32_t p_x10 = (int32_t)robot_ctrl.supercap.chassis_output_power;
                // 当电容无数据时，降级使用裁判系统底盘功率
                if (p_x10 == 0) {
                    p_x10 = (int32_t)(robot_ctrl.referee_info.power_heat_data.reserved3 * 10.0f);
                }
                if (p_x10 < 0) p_x10 = -p_x10;

                uint16_t p_clamp = clamp_u16(p_x10, UI_POWER_MIN_X10, UI_POWER_MAX_X10);
                uint16_t p_fill_len = (uint16_t)(((uint32_t)(p_clamp - UI_POWER_MIN_X10) * inner_span) /
                                                (uint32_t)(UI_POWER_MAX_X10 - UI_POWER_MIN_X10));
                if ((p_clamp > 0U) && (p_fill_len == 0U)) p_fill_len = 1U;
                uint16_t p_fill_right = bar_inner_left + p_fill_len;

                make_rect(&status_figs5[2], 'P', 'W', '0', op_status, REF_UI_COLOR_WHITE,
                          bar_left, UI_PWR_BAR_Y - 20U, bar_right, UI_PWR_BAR_Y + 20U, 2U);
                make_line(&status_figs5[3], 'P', 'W', '1', op_status, REF_UI_COLOR_YELLOW,
                          bar_inner_left, UI_PWR_BAR_Y, p_fill_right, UI_PWR_BAR_Y, 12U);

                // 中间车体准心框
                make_rect(&status_figs5[4], 'C', 'M', '0', op_status, REF_UI_COLOR_WHITE,
                          UI_CAR_X0, UI_CAR_Y0, UI_CAR_X1, UI_CAR_Y1, 3U);

                // ---------- 绘制小车俯视图与装甲受击状态 ----------
                uint16_t car_cx = (UI_CAR_X0 + UI_CAR_X1) / 2U;
                uint16_t car_cy = (UI_CAR_Y0 + UI_CAR_Y1) / 2U;
                uint16_t half_w = (UI_CAR_X1 - UI_CAR_X0) / 2U;
                uint16_t half_h = (UI_CAR_Y1 - UI_CAR_Y0) / 2U;
                uint16_t out = 14U;

                make_line(&armor_figs5[0], 'A', 'F', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[0]),
                          car_cx - 26U, car_cy + half_h + out, car_cx + 26U, car_cy + half_h + out, 10U); // 前
                make_line(&armor_figs5[1], 'A', 'L', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[1]),
                          car_cx - half_w - out, car_cy - 26U, car_cx - half_w - out, car_cy + 26U, 10U); // 左
                make_line(&armor_figs5[2], 'A', 'B', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[2]),
                          car_cx - 26U, car_cy - half_h - out, car_cx + 26U, car_cy - half_h - out, 10U); // 后
                make_line(&armor_figs5[3], 'A', 'R', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[3]),
                          car_cx + half_w + out, car_cy - 26U, car_cx + half_w + out, car_cy + 26U, 10U); // 右

                // 占位中心点，保证能凑够5个图形满足 0x0103 协议
                make_line(&armor_figs5[4], 'A', 'D', '0', op_armor, REF_UI_COLOR_GREEN,
                          car_cx, car_cy, car_cx + 1U, car_cy + 1U, 1U);


                // ---------- 交替发送避免超带宽 ----------
                if (send_selector == 0U) {
                    UI_Draw_5(sender_id, receiver_id, status_figs5);
                    status_drawn_once = 1U;
                } else {
                    UI_Draw_5(sender_id, receiver_id, armor_figs5);
                    armor_drawn_once = 1U;
                }

                send_selector ^= 1U; // 翻转 0 和 1
                last_send_tick = osKernelSysTick();
            }
        }

        // 挂起 10ms (100Hz 刷新率)
        osDelay(10);
    }
}