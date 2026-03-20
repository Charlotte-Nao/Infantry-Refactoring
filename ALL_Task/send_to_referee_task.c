#include "send_to_referee_task.h"
// 【修改点 1】引入你真实的全局结构体和底层解析任务
#include "../Application/robot_global.h"
#include "../ALL_Task/analyze_from_referee_task.h"
#include "../Application/referee.h"

// 【修改点 2】引入你的 BSP 串口打印文件
#include "../Bsp/uart/bsp_uart.h"

#include "cmsis_os.h"
#include <string.h>

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

// 声明在 analyze_from_referee_task.c 中的串口接收计数器，用于调试
extern uint32_t uart6_rx_count;

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
    (void)argument;

    // 【修改点 3】获取调试串口 1 设备
    struct uart_device *debug_uart = uart_get_device("uart1_dma");
    uint32_t last_print_tick = osKernelSysTick();

    while (1) {
        // 【核心修改】指向你自己的真实数据源，不要用 global_info
        const referee_info_t *ref = &robot_ctrl.referee_info;

        uint16_t sender_id = ref->robot_status.robot_id;
        uint16_t receiver_id = Referee_Get_ClientId_By_RobotId(sender_id);

        // ========== 串口调试信息打印 (每秒打印一次，防止刷屏) ==========
        // if (osKernelSysTick() - last_print_tick >= 1000U) {
        //     if (debug_uart != NULL && debug_uart->Print != NULL) {
        //         debug_uart->Print(debug_uart, "[UI Task] Rx_Count: %d, sender_id: %d, receiver_id: 0x%04X, HP: %d\r\n",
        //                           uart6_rx_count, sender_id, receiver_id, ref->robot_status.current_HP);
        //     }
        //     last_print_tick = osKernelSysTick();
        // }
        // ===============================================================

        if ((sender_id != 0U) && (receiver_id != 0U))
        {
            if ((sender_id != last_sender_id) || (receiver_id != last_receiver_id))
            {
                cleared_once = 0U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_sender_id = sender_id;
                last_receiver_id = receiver_id;
            }

            if (cleared_once == 0U)
            {
                Referee_UI_Delete(sender_id, receiver_id, 2U, 0U);
                cleared_once = 1U;
                status_drawn_once = 0U;
                armor_drawn_once = 0U;
                send_selector = 0U;
                last_send_tick = osKernelSysTick();
            }

            if ((osKernelSysTick() - last_send_tick) >= UI_SEND_PERIOD_MS)
            {
                uint32_t now = osKernelSysTick();
                uint8_t op_status = (status_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;
                uint8_t op_armor = (armor_drawn_once == 0U) ? REF_UI_OP_ADD : REF_UI_OP_MODIFY;
                uint8_t tx_ok;
                interaction_figure_param_t status_figs5[5];
                interaction_figure_param_t armor_figs5[5];

                // 受击高亮计算
                uint8_t armor_id = ref->huart_robot.armor_id;
                uint8_t reason = ref->huart_robot.HP_deducation_reason;
                int8_t armor_idx = armor_id_to_index(armor_id);
                uint8_t sig = (uint8_t)((reason << 4) | (armor_id & 0x0FU));
                uint16_t hp_now = ref->robot_status.current_HP;
                uint8_t hp_drop = 0U;

                if ((last_hp != 0xFFFFU) && (hp_now < last_hp)) hp_drop = 1U;
                last_hp = hp_now;

                if (((sig != last_hurt_sig) || (hp_drop != 0U)) && (armor_idx >= 0)) {
                    armor_hit_tick[(uint8_t)armor_idx] = now;
                    last_hurt_sig = sig;
                }

                // 超级电容进度条
                int16_t v = robot_ctrl.supercap.capacity_voltage;
                uint16_t v_clamp = clamp_u16((int32_t)v, (uint16_t)UI_CAP_VOLTAGE_MIN_X100, (uint16_t)UI_CAP_VOLTAGE_MAX_X100);
                uint16_t bar_left = UI_STATUS_BAR_LEFT;
                uint16_t bar_right = UI_STATUS_BAR_RIGHT;
                uint16_t bar_y = UI_CAP_BAR_Y;
                uint16_t bar_inner_left = (uint16_t)(bar_left + 8U);
                uint16_t bar_inner_right_max = (uint16_t)(bar_right - 8U);
                uint16_t inner_span = (uint16_t)(bar_inner_right_max - bar_inner_left);
                uint16_t fill_len = (uint16_t)(((uint32_t)(v_clamp - UI_CAP_VOLTAGE_MIN_X100) * inner_span) /
                                                (uint32_t)(UI_CAP_VOLTAGE_MAX_X100 - UI_CAP_VOLTAGE_MIN_X100));
                uint16_t fill_right = (uint16_t)(bar_inner_left + fill_len);

                make_rect(&status_figs5[0], 'C', 'B', '0', op_status, REF_UI_COLOR_WHITE,
                          bar_left, (uint16_t)(bar_y - 20U), bar_right, (uint16_t)(bar_y + 20U), 2U);
                make_line(&status_figs5[1], 'C', 'B', '1', op_status, REF_UI_COLOR_GREEN,
                          bar_inner_left, bar_y, fill_right, bar_y, 12U);

                // 底盘实时功率
                int32_t p_x10 = (int32_t)robot_ctrl.supercap.chassis_output_power;
                if (p_x10 == 0) p_x10 = (int32_t)(ref->power_heat_data.reserved3 * 10.0f);
                if (p_x10 < 0) p_x10 = -p_x10;

                uint16_t p_clamp = clamp_u16(p_x10, (uint16_t)UI_POWER_MIN_X10, (uint16_t)UI_POWER_MAX_X10);
                uint16_t p_fill_len = (uint16_t)(((uint32_t)(p_clamp - UI_POWER_MIN_X10) * inner_span) /
                                                (uint32_t)(UI_POWER_MAX_X10 - UI_POWER_MIN_X10));
                if ((p_clamp > 0U) && (p_fill_len == 0U)) p_fill_len = 1U;
                uint16_t p_fill_right = (uint16_t)(bar_inner_left + p_fill_len);

                make_rect(&status_figs5[2], 'P', 'W', '0', op_status, REF_UI_COLOR_WHITE,
                          bar_left, (uint16_t)(UI_PWR_BAR_Y - 20U), bar_right, (uint16_t)(UI_PWR_BAR_Y + 20U), 2U);
                make_line(&status_figs5[3], 'P', 'W', '1', op_status, REF_UI_COLOR_YELLOW,
                          bar_inner_left, UI_PWR_BAR_Y, p_fill_right, UI_PWR_BAR_Y, 12U);

                make_rect(&status_figs5[4], 'C', 'M', '0', op_status, REF_UI_COLOR_WHITE,
                          UI_CAR_X0, UI_CAR_Y0, UI_CAR_X1, UI_CAR_Y1, 3U);

                // 小车俯视图
                uint16_t car_cx = (uint16_t)((UI_CAR_X0 + UI_CAR_X1) / 2U);
                uint16_t car_cy = (uint16_t)((UI_CAR_Y0 + UI_CAR_Y1) / 2U);
                uint16_t half_w = (uint16_t)((UI_CAR_X1 - UI_CAR_X0) / 2U);
                uint16_t half_h = (uint16_t)((UI_CAR_Y1 - UI_CAR_Y0) / 2U);
                uint16_t out = 14U;

                make_line(&armor_figs5[0], 'A', 'F', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[0]),
                          (uint16_t)(car_cx - 26U), (uint16_t)(car_cy + half_h + out),
                          (uint16_t)(car_cx + 26U), (uint16_t)(car_cy + half_h + out), 10U);
                make_line(&armor_figs5[1], 'A', 'L', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[1]),
                          (uint16_t)(car_cx - half_w - out), (uint16_t)(car_cy - 26U),
                          (uint16_t)(car_cx - half_w - out), (uint16_t)(car_cy + 26U), 10U);
                make_line(&armor_figs5[2], 'A', 'B', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[2]),
                          (uint16_t)(car_cx - 26U), (uint16_t)(car_cy - half_h - out),
                          (uint16_t)(car_cx + 26U), (uint16_t)(car_cy - half_h - out), 10U);
                make_line(&armor_figs5[3], 'A', 'R', '0', op_armor, armor_color_by_tick(now, armor_hit_tick[3]),
                          (uint16_t)(car_cx + half_w + out), (uint16_t)(car_cy - 26U),
                          (uint16_t)(car_cx + half_w + out), (uint16_t)(car_cy + 26U), 10U);
                make_line(&armor_figs5[4], 'A', 'D', '0', op_armor, REF_UI_COLOR_GREEN,
                          car_cx, car_cy, (uint16_t)(car_cx + 1U), (uint16_t)(car_cy + 1U), 1U);

                if (send_selector == 0U) {
                    tx_ok = Referee_UI_Draw5(sender_id, receiver_id, status_figs5);
                    if (tx_ok != 0U) status_drawn_once = 1U;
                } else {
                    tx_ok = Referee_UI_Draw5(sender_id, receiver_id, armor_figs5);
                    if (tx_ok != 0U) armor_drawn_once = 1U;
                }

                send_selector ^= 1U;
                last_send_tick = osKernelSysTick();
            }
        }
        osDelay(10);
    }
}