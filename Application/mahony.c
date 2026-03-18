//
// Created by 14717 on 2025/9/2.
//

#include "mahony.h"


#include <math.h>

#include "../Algorithm/MahonyAHRS/MahonyAHRS.h"

/**********************************************************************************************************************/
/*更新四元数*/
void pos_update(fp32 quat[4], fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    //MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
    /*因为磁力计不准，所以改用下面的姿态融合结算函数*/
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

/**********************************************************************************************************************/
/*解算姿态角*/
void get_angle(fp32 q[4], fp32 angle[3])
{
    angle[0] = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
    angle[1] = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]));
    angle[2] = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f);
}