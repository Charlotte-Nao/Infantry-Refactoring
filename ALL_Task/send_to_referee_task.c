#include "send_to_referee_task.h"
// 【修改点 1】引入你真实的全局结构体和底层解析任务
#include "../Application/robot_global.h"
#include "../ALL_Task/analyze_from_referee_task.h"
#include "../Application/referee.h"

// 【修改点 2】引入你的 BSP 串口打印文件
#include "../Bsp/uart/bsp_uart.h"

#include "cmsis_os.h"
#include <string.h>

/* ==================== 高级自定义中置 UI 宏定义 ==================== */
/* ==================== UI 布局宏定义 ==================== */
#define UI_SEND_PERIOD_MS            100U
#define UI_HIT_HIGHLIGHT_MS          300U
#define UI_CAP_VOLTAGE_MIN_X100      0
#define UI_CAP_VOLTAGE_MAX_X100      3000
#define UI_POWER_MIN_X10             0
#define UI_POWER_MAX_X10             2000

// 经典左上角状态区布局 (保留原样)
#define UI_STATUS_BAR_LEFT           50U
#define UI_STATUS_BAR_RIGHT          520U
#define UI_CAP_BAR_Y                 860U
#define UI_PWR_BAR_Y                 800U

// 中置准星布局 (1080P 屏幕中央)
#define SCREEN_CENTER_X              960U
#define SCREEN_CENTER_Y              540U
#define UI_CAR_X0                    950U
#define UI_CAR_Y0                    530U
#define UI_CAR_X1                    970U
#define UI_CAR_Y1                    550U

//受击位置布局
#define UI_HURT_DISTANCE             250U
#define UI_HURT_LENGTH               100U
#define UI_HURT_WIDTH                4U

// 声明在 analyze_from_referee_task.c 中的串口接收计数器，用于调试
extern uint32_t uart6_rx_count;

static uint16_t clamp_u16(int32_t v, uint16_t lo, uint16_t hi) {
    if (v < (int32_t)lo) return lo;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

static uint8_t armor_color_by_tick(uint32_t now, uint32_t last_hit_tick) {
    return ((now - last_hit_tick) <= UI_HIT_HIGHLIGHT_MS) ? REF_UI_COLOR_PINK : REF_UI_COLOR_GREEN;
}

static int8_t armor_id_to_index(uint8_t armor_id) {
    if (armor_id <= 3U) return (int8_t)armor_id;
    if (armor_id == 4U) return 3;
    return -1;
}

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
    uint32_t armor_hit_tick[4] = {0U, 0U, 0U, 0U};

    while (1) {
        uint16_t sender_id = robot_ctrl.referee_info.robot_status.robot_id;
        uint16_t receiver_id = 0;

        if (sender_id != 0U) {
            receiver_id = Referee_Get_ClientId_By_RobotId(sender_id);

            if ((sender_id != last_sender_id) || (receiver_id != last_receiver_id)) {
                cleared_once = 0U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_sender_id = sender_id;
                last_receiver_id = receiver_id;
            }

            if (receiver_id != 0U) {
                if (cleared_once == 0U) {
                    Referee_UI_Delete(sender_id, receiver_id, 2, 0);
                    cleared_once = 1U;
                    status_drawn_once = 0U;
                    armor_drawn_once = 0U;
                    send_selector = 0U;
                    last_send_tick = osKernelSysTick();
                }

                if ((osKernelSysTick() - last_send_tick) >= UI_SEND_PERIOD_MS) {
                    uint32_t now = osKernelSysTick();
                    uint8_t op_status = (status_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;
                    uint8_t op_armor = (armor_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;

                    interaction_figure_param_t status_figs5[5];
                    interaction_figure_param_t armor_figs5[5];

                    // ==================== 受击高亮与血量监控 ====================
                    uint8_t armor_id = robot_ctrl.referee_info.huart_robot.armor_id;
                    uint8_t reason = robot_ctrl.referee_info.huart_robot.HP_deducation_reason;
                    int8_t armor_idx = armor_id_to_index(armor_id);
                    uint8_t sig = (uint8_t)((reason << 4) | (armor_id & 0x0FU));
                    uint16_t hp_now = robot_ctrl.referee_info.robot_status.current_HP;
                    uint8_t hp_drop = 0U;

                    if ((last_hp != 0xFFFFU) && (hp_now < last_hp)) hp_drop = 1U;
                    last_hp = hp_now;

                    if (((sig != last_hurt_sig) || (hp_drop != 0U)) && (armor_idx >= 0)) {
                        armor_hit_tick[(uint8_t)armor_idx] = now;
                        last_hurt_sig = sig;
                    }

                    // ==================== 图形打包交替发送 ====================

                    if (send_selector == 0U) {
                        // 【PACK 0：原样左上角状态条 + 中置准星框】
                        uint16_t bar_left = UI_STATUS_BAR_LEFT;
                        uint16_t bar_right = UI_STATUS_BAR_RIGHT;
                        uint16_t bar_inner_left = bar_left + 8U;
                        uint16_t inner_span = (bar_right - 8U) - bar_inner_left;

                        // 1. 电容条计算与绘制
                        int16_t v = robot_ctrl.supercap.capacity_voltage;
                        uint16_t v_clamp = clamp_u16((int32_t)v, UI_CAP_VOLTAGE_MIN_X100, UI_CAP_VOLTAGE_MAX_X100);
                        uint16_t fill_len = (uint16_t)(((uint32_t)(v_clamp - UI_CAP_VOLTAGE_MIN_X100) * inner_span) /
                                                        (uint32_t)(UI_CAP_VOLTAGE_MAX_X100 - UI_CAP_VOLTAGE_MIN_X100));

                        make_rect(&status_figs5[0], 'C', 'B', '0', op_status, REF_UI_COLOR_WHITE,
                                  bar_left, UI_CAP_BAR_Y - 20U, bar_right, UI_CAP_BAR_Y + 20U, 2U);
                        make_line(&status_figs5[1], 'C', 'B', '1', op_status, REF_UI_COLOR_GREEN,
                                  bar_inner_left, UI_CAP_BAR_Y, bar_inner_left + fill_len, UI_CAP_BAR_Y, 12U);

                        // 2. 功率条计算与绘制
                        int32_t p_x10 = (int32_t)robot_ctrl.supercap.chassis_output_power;
                        if (p_x10 == 0) p_x10 = (int32_t)(robot_ctrl.referee_info.power_heat_data.reserved3 * 10.0f);
                        if (p_x10 < 0) p_x10 = -p_x10;

                        uint16_t p_clamp = clamp_u16(p_x10, UI_POWER_MIN_X10, UI_POWER_MAX_X10);
                        uint16_t p_fill_len = (uint16_t)(((uint32_t)(p_clamp - UI_POWER_MIN_X10) * inner_span) /
                                                        (uint32_t)(UI_POWER_MAX_X10 - UI_POWER_MIN_X10));
                        if ((p_clamp > 0U) && (p_fill_len == 0U)) p_fill_len = 1U;

                        make_rect(&status_figs5[2], 'P', 'W', '0', op_status, REF_UI_COLOR_WHITE,
                                  bar_left, UI_PWR_BAR_Y - 20U, bar_right, UI_PWR_BAR_Y + 20U, 2U);
                        make_line(&status_figs5[3], 'P', 'W', '1', op_status, REF_UI_COLOR_YELLOW,
                                  bar_inner_left, UI_PWR_BAR_Y, bar_inner_left + p_fill_len, UI_PWR_BAR_Y, 12U);

                        // 3. 绘制中间准星方框
                        make_rect(&status_figs5[4], 'C', 'M', '0', op_status, REF_UI_COLOR_CYAN,
                                  UI_CAR_X0, UI_CAR_Y0, UI_CAR_X1, UI_CAR_Y1, 2U);

                        Referee_UI_Draw5(sender_id, receiver_id, status_figs5);
                        status_drawn_once = 1U;

                    } else {
                        // 【PACK 1：巨型中置装甲受击框】
                        uint16_t cx = SCREEN_CENTER_X;
                        uint16_t cy = SCREEN_CENTER_Y;

                        // ========== 尺寸放大参数 ==========
                        uint16_t out = UI_HURT_DISTANCE ;   // 偏移半径: 控制受击框离中心有多远 (以前只有25)
                        uint16_t len = UI_HURT_LENGTH ;   // 线条半长: 控制每条受击线段有多长 (以前只有30)
                        uint16_t thick = UI_HURT_WIDTH ;  // 线条粗细: 让受击警告极度显眼 (以前只有5)

                        make_line(&armor_figs5[0], 'A', 'F', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[0]),
                                  cx - len, cy + out, cx + len, cy + out, thick); // 前 (上方)
                        make_line(&armor_figs5[1], 'A', 'L', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[1]),
                                  cx - out, cy - len, cx - out, cy + len, thick); // 左 (左方)
                        make_line(&armor_figs5[2], 'A', 'B', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[2]),
                                  cx - len, cy - out, cx + len, cy - out, thick); // 后 (下方)
                        make_line(&armor_figs5[3], 'A', 'R', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[3]),
                                  cx + out, cy - len, cx + out, cy + len, thick); // 右 (右方)

                        // 占位图形，填满5个
                        make_line(&armor_figs5[4], 'A', 'D', '0', op_armor, REF_UI_COLOR_GREEN, cx, cy, cx + 1U, cy + 1U, 1U);

                        Referee_UI_Draw5(sender_id, receiver_id, armor_figs5);
                        armor_drawn_once = 1U;
                    }

                    // 每 100ms 在 Pack 0 和 Pack 1 之间翻转
                    send_selector ^= 1U;
                    last_send_tick = osKernelSysTick();
                }
            }
        }
        osDelay(10);
    }
}