#ifndef IST8310_MIDDLEWARE_H
#define IST8310_MIDDLEWARE_H

#include "main.h"

/* IST8310 I2C 从机地址 */
#define IST8310_IIC_ADDRESS 0x0E

/**
 * @brief IST8310 硬件初始化（复位管脚等）
 */
void ist8310_GPIO_init(void);

/**
 * @brief IST8310 复位（硬件复位）
 */
void ist8310_reset_L(void);
void ist8310_reset_H(void);

/**
 * @brief I2C 读写封装
 */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#endif