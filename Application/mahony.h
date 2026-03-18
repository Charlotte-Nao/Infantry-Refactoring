//
// Created by 14717 on 2025/9/2.
//

#ifndef INFANTRY_01_MAHONY_H
#define INFANTRY_01_MAHONY_H

#include "struct_typedef.h"

void pos_update(fp32 quat[4], fp32 gyro[3], fp32 accel[3], fp32 mag[3]);

void get_angle(fp32 q[4], fp32 angle[3]);

#endif //INFANTRY_01_MAHONY_H