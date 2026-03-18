//
// Created by 14717 on 2025/8/27.
//

#ifndef INFANTRY_01_CAN_CONTROL_MOTOR_H
#define INFANTRY_01_CAN_CONTROL_MOTOR_H

/**********************************************************************************************************************/
/*宏定义*/

/*ID定义*/
#define DM_4310_MASTER_ID 0x00
#define CAN_J4310_PITCH_ID 0x01

#define CAN_M3508_CHASSIS_1_ID 0x201
#define CAN_M3508_CHASSIS_2_ID 0x202
#define CAN_M3508_CHASSIS_3_ID 0x203
#define CAN_M3508_CHASSIS_4_ID 0x204
#define CAN_M2006_TRIGGER_ID 0x205
#define CAN_GM6020_YAW_ID 0x206
#define CAN_M3508_SHOOT_R_ID 0x207
#define CAN_M3508_SHOOT_L_ID 0x208

extern void bsp_can_init(void);

#endif //INFANTRY_01_CAN_CONTROL_MOTOR_H