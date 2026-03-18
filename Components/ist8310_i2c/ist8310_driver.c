#include "ist8310_driver.h"
#include "ist8310_middleware.h"
#include "cmsis_os.h"

/* 灵敏度 0.3uT/LSB */
#define MAG_SEN 0.3f

#define IST8310_WRITE_REG_NUM 4
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01}, // 开启中断，低电平触发
    {0x41, 0x09, 0x02}, // 平均采样2次
    {0x42, 0xC0, 0x03}, // 必须设为0xC0
    {0x0A, 0x0B, 0x04}  // 200Hz ODR
};

uint8_t ist8310_init(void) {
    uint8_t res = 0;
    uint8_t writeNum = 0;

    // 1. 硬件复位
    ist8310_reset_L();
    vTaskDelay(50);
    ist8310_reset_H();
    vTaskDelay(50);

    // 2. 读取 ID 确认
    res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
    if (res != IST8310_WHO_AM_I_VALUE) {
        return IST8310_NO_SENSOR;
    }

    // 3. 循环配置寄存器并校验
    for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
        ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                     ist8310_write_reg_data_error[writeNum][1]);
        vTaskDelay(10); // 缩短等待时间，原150ms太浪费性能

        res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
        if (res != ist8310_write_reg_data_error[writeNum][1]) {
            return ist8310_write_reg_data_error[writeNum][2];
        }
    }
    return IST8310_NO_ERROR;
}

void ist8310_read_mag(fp32 mag[3]) {
    uint8_t buf[6];
    int16_t temp;

    // 从 0x03 寄存器开始连续读取 6 个字节 (X, Y, Z)
    ist8310_IIC_read_muli_reg(0x03, buf, 6);

    temp = (int16_t)((buf[1] << 8) | buf[0]);
    mag[0] = MAG_SEN * temp;

    temp = (int16_t)((buf[3] << 8) | buf[2]);
    mag[1] = MAG_SEN * temp;

    temp = (int16_t)((buf[5] << 8) | buf[4]);
    mag[2] = MAG_SEN * temp;
}