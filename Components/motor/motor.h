#ifndef INFANTRY_01_MOTOR_H
#define INFANTRY_01_MOTOR_H

#include "../../Application/struct_typedef.h"
#include "stm32f4xx_hal.h"

/**
 * @struct motor_device
 * @brief 电机统一抽象接口结构体
 */
struct motor_device {
    // 成员变量
    char *motor_name;               /*!< 电机实例名称 */
    uint32_t motor_id;              /*!< 反馈帧对应的 StdId (大疆: 0x201+; 达妙: 0x00) */
    CAN_HandleTypeDef* motor_can_handle; /*!< 电机挂载的 CAN 句柄 */
    void *motor_data;               /*!< 指向私有数据结构体 (如 DM_MIT_data, M3508_data) */

    // --- 函数指针接口 ---

    /**
     * @brief 初始化电机配置
     * @param motor 电机句柄
     * @param motor_ID CAN标准ID
     * @param hcan CAN总线句柄
     * @param para_num 额外参数个数
     * @param ... 可变参数说明:
     * - **达妙MIT**: (double kp, double kd, double P_MAX, double V_MAX, double T_MAX)
     * - **大疆3508/2006**: (double Kp, double Ki, double Kd, double out_max, double i_max, double alpha)
     * - **大疆6020**: (double P_Kp, double P_Ki, double V_Kp, double V_Ki, double V_Kd, double out_max, double v_limit, double alpha)
     */
    void (*init)(struct motor_device *motor, uint32_t motor_ID, CAN_HandleTypeDef *hcan, int para_num, ...);

    /**
     * @brief 解析原始 CAN 报文并更新反馈量
     * @param motor 电机句柄
     * @param data CAN 接收到的 8 字节数据指针
     */
    void (*get_measure)(const struct motor_device *motor, const uint8_t *data);

    /**
     * @brief 更新电机内部控制算法（如 PID 计算）
     * @note 建议在控制任务中以恒定频率调用
     */
    void (*update)(struct motor_device *motor);

    /**
     * @brief 发送使能指令（进入工作模式）
     */
    void (*send_enable_cmd)(struct motor_device *motor);

    /**
     * @brief 发送失能指令（电机停止输出且进入安全状态）
     */
    void (*send_disable_cmd)(struct motor_device *motor);

    /**
     * @brief 执行实际的 CAN 组帧发送动作
     * @note 大疆电机通常通过专用 Group 发送函数批量发送，此函数可能为空
     */
    void (*send_ctrl_cmd)(struct motor_device *motor);

    /**
     * @brief 设置电机运行的目标值
     * @param motor 电机句柄
     * @param para_num 参数个数
     * @param ... 可变参数说明:
     * - **大疆3508/2006 (速度模式)**: (double v_des_rpm)
     * - **大疆6020 (位置速度模式)**: (double p_des_rad)
     * - **达妙MIT模式**: (double p_des, double v_des, double t_ff)
     */
    void (*set_target)(const struct motor_device *motor, const int para_num, ...);

    /**
     * @brief 获取电机运行状态或参数
     * @param motor 电机句柄
     * @param which_status 状态名字符串:
     * - 通用: "POS" (位置), "VEL" (速度), "ERR" (错误码), "TEMP" (温度)
     * - 目标: "p_des", "v_des"
     * - PID参数: "Kp", "Ki", "Kd"
     * @param status_data 接收数据的指针 (需根据具体类型强制转换)
     */
    void (*get_status)(const struct motor_device *motor, const char* which_status, void* status_data);

    /**
     * @brief 运行时动态修改参数
     * @param motor 电机句柄
     * @param which_para 参数名字符串 (如 "Kp", "Ki", "V_MAX")
     * @param para_data 指向新参数值的指针
     */
    void (*set_para)(const struct motor_device *motor, const char* which_para, void* para_data);
};

/* --- 电机控制核心接口 --- */

/**
 * @brief 根据名称在电机池中查找对应的电机实例
 * @param name 注册时的名称 (如 "M3508_CHASSIS_1")
 * @return 指向电机实例的指针，未找到返回 NULL
 */
struct motor_device *motor_get_device(const char *name);

/**
 * @brief 系统启动时初始化所有已注册电机
 */
void Motor_System_PowerOn_Init(void);

/**
 * @brief 遍历并调用所有已注册电机的 update 函数
 */
void Motor_All_Update(void);

/**
 * @brief 将 CAN1 总线上所有大疆电机的电流控制指令打包发送
 */
void DJI_Motor_Send_CAN1_Group(CAN_HandleTypeDef *hcan);

/**
 * @brief 将 CAN2 总线上所有大疆电机的电流控制指令打包发送
 */
void DJI_Motor_Send_CAN2_Group(CAN_HandleTypeDef *hcan);

// 硬件句柄声明
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#endif //INFANTRY_01_MOTOR_H