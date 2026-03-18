#ifndef IST8310_DRIVER_H
#define IST8310_DRIVER_H

#include "main.h"

#define IST8310_WHO_AM_I 0x00
#define IST8310_WHO_AM_I_VALUE 0x10

/* 错误码定义 */
#define IST8310_NO_ERROR 0
#define IST8310_NO_SENSOR 1

typedef float fp32;

uint8_t ist8310_init(void);
void ist8310_read_mag(fp32 mag[3]);

#endif