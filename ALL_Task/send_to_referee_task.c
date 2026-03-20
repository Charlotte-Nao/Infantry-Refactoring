//
// Created by ASUS on 2026/3/19.
//

#include "send_to_referee_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "string.h"

// 1. 引入系统全局变量 (包含 robot_ctrl_info_t 结构体)
#include "..robot_global.h"

// 2. 引入分离出去的 UI 绘制底层接口
#include "referee.h"

/* ==================== UI 布局相关宏定义 ==================== */
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


/* ==================== 内部工具函数 ==================== */

// 限制数值范围
static uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

// 根据时间戳判断装甲板颜色（被击打时变为橙色）
static uint8_t armor_color_by_tick(uint32_t now, uint32_t last_hit_tick) {
    return ((now - last_hit_tick) <= UI_HIT_HIGHLIGHT_MS) ? REF_UI_COLOR_ORANGE : REF_UI_COLOR_GREEN;
}

// 将装甲板 ID 转换为数组索引
static int8_t armor_id_to_index(uint8_t armor_id) {
    if (armor_id <= 3U) return (int8_t)armor_id;
    if (armor_id == 4U) return 3;
    return -1;
}

// 构造矩形数据 (写入 interaction_figure_param_t)
static void make_rect(interaction_figure_param_t *fig, const char name0, const char name1, const char name2,
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

// 构造线条数据
static void make_line(interaction_figure_param_t *fig, const char name0, const char name1, const char name2,
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
        // 直接从你的全局结构体 robot_ctrl 中拉取本机 ID
        uint16_t sender_id = robot_ctrl.referee_info.robot_status.robot_id;
        uint16_t receiver_id = 0;

        // 只有当接收到真实的机器人 ID 时才执行 UI 发送逻辑
        if (sender_id != 0U) {

            // 使用 referee.c 提供的标准函数计算选手端 ID
            receiver_id = Referee_Get_ClientId_By_RobotId(sender_id);

            // 如果机器人 ID 变化 (比如重启/掉线重连) 时，重置标志位
            if ((sender_id != last_sender_id) || (receiver_id != last_receiver_id)) {
                cleared_once = 0U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_sender_id = sender_id;
                last_receiver_id = receiver_id;
            }

            // 安全检查，若不是合法的选手端就不发
            if (receiver_id != 0U) {
                // 1. 初次连接，清除屏幕所有图层残影
                if (cleared_once == 0U) {
                    Referee_UI_Delete(sender_id, receiver_id, 2, 0); // 2:删除所有
                    cleared_once = 1U;
                    status_drawn_once = 0U;
                    armor_drawn_once = 0U;
                    send_selector = 0U;
                    last_send_tick = osKernelSysTick();
                }

                // 2. 10Hz (100ms) 频率交替发送状态栏和装甲受击指示
                if ((osKernelSysTick() - last_send_tick) >= UI_SEND_PERIOD_MS) {
                    uint32_t now = osKernelSysTick();

                    // 判断是首次添加(ADD)还是后续修改(MODIFY)
                    uint8_t op_status = (status_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;
                    uint8_t op_armor = (armor_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;

                    interaction_figure_param_t status_figs5[5];
                    interaction_figure_param_t armor_figs5[5];

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

                    // 触发受击高亮条件：受到伤害或掉血
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

                    // 当电容无数据时，降级使用裁判系统传入的保留底盘功率
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

                    // 占位中心点，保证能凑够 5 个图形满足 0x0103 协议
                    make_line(&armor_figs5[4], 'A', 'D', '0', op_armor, REF_UI_COLOR_GREEN,
                              car_cx, car_cy, car_cx + 1U, car_cy + 1U, 1U);


                    // ---------- 交替发送以节省串口和裁判系统带宽 ----------
                    if (send_selector == 0U) {
                        Referee_UI_Draw5(sender_id, receiver_id, status_figs5);
                        status_drawn_once = 1U;
                    } else {
                        Referee_UI_Draw5(sender_id, receiver_id, armor_figs5);
                        armor_drawn_once = 1U;
                    }

                    send_selector ^= 1U; // 在 0 和 1 之间翻转
                    last_send_tick = osKernelSysTick();
                }
            }
        }

        // 挂起 10ms (100Hz 刷新率)
        osDelay(10);
    }
}