#ifndef __IMU_CONTROL_H
#define __IMU_CONTROL_H

#include "Struct_encapsulation.h"
#include "pid.h"
#include "IMU.h"
#include "motor_command.h"

typedef struct IMU_RUNDATA
{
    float ANGLE;
    float LAST_ANGLE;
    float IS_MOVING;
};

void Gyro_Init(void);
float getAngleZ(float yaw, float my_angle);
float getAngleZ_avg(float my_angle);
float normalizeAngle(float angle);
void Direction_Calibration(float target_angle);
float Direction_Calibration_turn(float target_angle);



#endif
