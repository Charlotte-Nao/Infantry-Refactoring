#include "ist8310_middleware.h"
#include "i2c.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c3;

void ist8310_GPIO_init(void) {
    // CubeMX 已生成 GPIOG 6 的初始化，此处可按需添加
}

void ist8310_reset_L(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
}

void ist8310_reset_H(void) {
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
}

void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

uint8_t ist8310_IIC_read_single_reg(uint8_t reg) {
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
    return res;
}

void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len) {
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}