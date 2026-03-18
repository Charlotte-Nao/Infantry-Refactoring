#include "motor_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "stdio.h"

/**
 * @brief 电机任务执行函数
 * @note  优先级：High (1ms)
 */
void motor_task_func(void const * argument) {
    // 1. 系统启动保护
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);
    Motor_System_PowerOn_Init();

    // 2. 获取所有电机句柄
    struct motor_device* pitch   = motor_get_device("J4310_PITCH");
    struct motor_device* yaw     = motor_get_device("GM6020_YAW");
    struct motor_device* shoot_l = motor_get_device("M3508_SHOOT_L");
    struct motor_device* shoot_r = motor_get_device("M3508_SHOOT_R");
    struct motor_device* stir_m  = motor_get_device("M2006_TRIGGER");
    struct motor_device* chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    // 3. 模式历史记录（用于边缘触发检测）
    static gimbal_mode_e  last_gimbal_mode  = GIMBAL_RELAX;
    static chassis_mode_e last_chassis_mode = CHASSIS_RELAX;
    static shoot_mode_e   last_shoot_mode   = SHOOT_STOP;

    while (1) {
        /* --- A. 边缘触发：云台使能控制 --- */
        if (robot_ctrl.gimbal_mode != last_gimbal_mode) {
            if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
                if(pitch) pitch->send_disable_cmd(pitch);
                if(yaw)   yaw->send_disable_cmd(yaw);
            } else {
                if(pitch) pitch->send_enable_cmd(pitch);
                if(yaw)   yaw->send_enable_cmd(yaw);
            }
            last_gimbal_mode = robot_ctrl.gimbal_mode;
        }

        /* --- B. 边缘触发：发射机构使能控制 --- */
        if (robot_ctrl.shoot_mode != last_shoot_mode) {
            if (robot_ctrl.shoot_mode == SHOOT_STOP) {
                if(shoot_l) shoot_l->send_disable_cmd(shoot_l);
                if(shoot_r) shoot_r->send_disable_cmd(shoot_r);
                if(stir_m)  stir_m->send_disable_cmd(stir_m);
            } else {
                if(shoot_l) shoot_l->send_enable_cmd(shoot_l);
                if(shoot_r) shoot_r->send_enable_cmd(shoot_r);
                if(stir_m)  stir_m->send_enable_cmd(stir_m);
            }
            last_shoot_mode = robot_ctrl.shoot_mode;
        }

        /* --- C. 边缘触发：底盘使能控制 --- */
        if (robot_ctrl.chassis_mode != last_chassis_mode) {
            for(int i=0; i<4; i++) {
                if(!chassis[i]) continue;
                if (robot_ctrl.chassis_mode == CHASSIS_RELAX)
                    chassis[i]->send_disable_cmd(chassis[i]);
                else
                    chassis[i]->send_enable_cmd(chassis[i]);
            }
            last_chassis_mode = robot_ctrl.chassis_mode;
        }

        /* --- D. 硬件指令下发 (每毫秒执行一次) --- */

        // 执行所有电机的计算回调（PID计算将 set_target 转为输出电流）
        Motor_All_Update();

        // 将控制电流发送至 CAN 总线
        DJI_Motor_Send_CAN1_Group(&hcan1);
        DJI_Motor_Send_CAN2_Group(&hcan2);

        // 达妙电机（Pitch轴）使用专用协议帧发送
        if(pitch) pitch->send_ctrl_cmd(pitch);

        osDelay(1);
    }
}