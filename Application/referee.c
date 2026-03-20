//
// Created by Gemini on 2026/3/20.
//

#include "referee.h"
#include "analyze_from_referee_task.h" // 引入底层发送函数 Referee_Send_Packet
#include <string.h>

// 获取对应的操作手客户端 ID (一般是机器人 ID + 0x0100)
uint16_t Referee_Get_ClientId_By_RobotId(uint16_t robot_id)
{
    // 1~6 为红方，101~106 为蓝方
    if ((robot_id >= 1 && robot_id <= 6) || (robot_id >= 101 && robot_id <= 106)) {
        return robot_id + 0x0100;
    }
    return 0; // 无效或不需要选手端的机器人
}

// 核心打包函数：拼接 6 字节的数据段头，并调用你的底层发送函数
static uint8_t Referee_Send_Interactive(uint16_t data_cmd_id, uint16_t sender_id, uint16_t receiver_id, const uint8_t *data, uint16_t data_len)
{
    // 数据段最大 112 字节，加上 6 字节的内容段头 = 118 字节
    if (data_len > 112) return 0;

    uint8_t payload[118] = {0};

    // 填充 6 字节内容段头
    payload[0] = data_cmd_id & 0xFF;
    payload[1] = (data_cmd_id >> 8) & 0xFF;
    payload[2] = sender_id & 0xFF;
    payload[3] = (sender_id >> 8) & 0xFF;
    payload[4] = receiver_id & 0xFF;
    payload[5] = (receiver_id >> 8) & 0xFF;

    // 填充具体的 UI 数据 (删除操作或图形数据)
    if (data_len > 0 && data != NULL) {
        memcpy(&payload[6], data, data_len);
    }

    // 0x0301 是机器人间交互数据的命令码
    Referee_Send_Packet(0x0301, payload, 6 + data_len);

    return 1;
}

// 将 13 字节的易读结构体，压缩位域到 15 字节的 RM 协议包
static void Referee_UI_Pack_Figure15(uint8_t out15[15], const interaction_figure_param_t *in)
{
    if (!out15 || !in) return;

    out15[0] = in->figure_name[0];
    out15[1] = in->figure_name[1];
    out15[2] = in->figure_name[2];

    // 位域压缩
    uint32_t cfg1 = (in->operate_type & 0x07) |
                    ((in->figure_type & 0x07) << 3) |
                    ((in->layer & 0x0F) << 6) |
                    ((in->color & 0x0F) << 10) |
                    ((in->details_a & 0x01FF) << 14) |
                    ((in->details_b & 0x01FF) << 23);

    uint32_t cfg2 = (in->width & 0x03FF) |
                    ((in->start_x & 0x07FF) << 10) |
                    ((in->start_y & 0x07FF) << 21);

    uint32_t cfg3 = (in->details_c & 0x03FF) |
                    ((in->details_d & 0x07FF) << 10) |
                    ((in->details_e & 0x07FF) << 21);

    // 采用 memcpy 避免内存对齐错误
    memcpy(&out15[3], &cfg1, 4);
    memcpy(&out15[7], &cfg2, 4);
    memcpy(&out15[11], &cfg3, 4);
}

// ======================== 对外发布的 UI 接口 ========================

uint8_t Referee_UI_Delete(uint16_t sender_id, uint16_t receiver_id, uint8_t delete_type, uint8_t layer) {
    interaction_layer_delete_t del = {delete_type, layer};
    return Referee_Send_Interactive(REF_UI_DATA_ID_DELETE, sender_id, receiver_id, (uint8_t *)&del, sizeof(del));
}

uint8_t Referee_UI_Draw1(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t *figure) {
    uint8_t packed[15];
    Referee_UI_Pack_Figure15(packed, figure);
    return Referee_Send_Interactive(REF_UI_DATA_ID_DRAW_1, sender_id, receiver_id, packed, sizeof(packed));
}

uint8_t Referee_UI_Draw2(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[2]) {
    uint8_t packed[30];
    for (int i = 0; i < 2; i++) Referee_UI_Pack_Figure15(&packed[i * 15], &figures[i]);
    return Referee_Send_Interactive(REF_UI_DATA_ID_DRAW_2, sender_id, receiver_id, packed, sizeof(packed));
}

uint8_t Referee_UI_Draw5(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[5]) {
    uint8_t packed[75];
    for (int i = 0; i < 5; i++) Referee_UI_Pack_Figure15(&packed[i * 15], &figures[i]);
    return Referee_Send_Interactive(REF_UI_DATA_ID_DRAW_5, sender_id, receiver_id, packed, sizeof(packed));
}

uint8_t Referee_UI_Draw7(uint16_t sender_id, uint16_t receiver_id, const interaction_figure_param_t figures[7]) {
    uint8_t packed[105];
    for (int i = 0; i < 7; i++) Referee_UI_Pack_Figure15(&packed[i * 15], &figures[i]);
    return Referee_Send_Interactive(REF_UI_DATA_ID_DRAW_7, sender_id, receiver_id, packed, sizeof(packed));
}