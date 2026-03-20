//
// Created by Gemini on 2026/3/20.
//

#ifndef INFANTRY_01_REFEREE_H
#define INFANTRY_01_REFEREE_H

#include <stdint.h>

/* --- 交互子内容 ID --- */
#define REF_UI_DATA_ID_DELETE   0x0100
#define REF_UI_DATA_ID_DRAW_1   0x0101
#define REF_UI_DATA_ID_DRAW_2   0x0102
#define REF_UI_DATA_ID_DRAW_5   0x0103
#define REF_UI_DATA_ID_DRAW_7   0x0104
#define REF_UI_DATA_ID_CHAR     0x0110

/* --- 图形操作/类型/颜色枚举 --- */
enum {
    REF_UI_OP_NULL = 0,
    REF_UI_OP_ADD = 1,
    REF_UI_OP_MODIFY = 2,
    REF_UI_OP_DELETE = 3,
};

enum {
    REF_UI_TYPE_LINE = 0,
    REF_UI_TYPE_RECT = 1,
    REF_UI_TYPE_CIRCLE = 2,
    REF_UI_TYPE_ELLIPSE = 3,
    REF_UI_TYPE_ARC = 4,
    REF_UI_TYPE_FLOAT = 5,
    REF_UI_TYPE_INT = 6,
    REF_UI_TYPE_CHAR = 7,
};

enum {
    REF_UI_COLOR_SELF = 0, // 红/蓝(己方颜色)
    REF_UI_COLOR_YELLOW = 1,
    REF_UI_COLOR_GREEN = 2,
    REF_UI_COLOR_ORANGE = 3,
    REF_UI_COLOR_PURPLE = 4,
    REF_UI_COLOR_PINK = 5,
    REF_UI_COLOR_CYAN = 6,
    REF_UI_COLOR_BLACK = 7,
    REF_UI_COLOR_WHITE = 8,
};

/* --- 客户端交互结构体 --- */
typedef struct __attribute__((packed)) {
    uint8_t delete_type;    // 0:空操作 1:删除图层 2:删除所有
    uint8_t layer;          // 图层 0~9
} interaction_layer_delete_t;

// 供应用层使用的图形参数结构体（未压缩，方便赋值）
typedef struct {
    uint8_t figure_name[3];
    uint8_t operate_type;
    uint8_t figure_type;
    uint8_t layer;
    uint8_t color;
    uint16_t details_a;
    uint16_t details_b;
    uint16_t width;
    uint16_t start_x;
    uint16_t start_y;
    uint16_t details_c;
    uint16_t details_d;
    uint16_t details_e;
} interaction_figure_param_t;

/* --- UI 接口函数声明 --- */

// 转换机器人 ID 为对应的选手端 ID
uint16_t Referee_Get_ClientId_By_RobotId(uint16_t robot_id);

// UI 绘制接口
uint8_t Referee_UI_Delete(uint16_t sender_id, uint16_t receiver_id, uint8_t delete_type, uint8_t layer);
uint8_t Referee_UI_Draw1(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t *figure);
uint8_t Referee_UI_Draw2(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[2]);
uint8_t Referee_UI_Draw5(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[5]);
uint8_t Referee_UI_Draw7(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[7]);

#endif //INFANTRY_01_REFEREE_H