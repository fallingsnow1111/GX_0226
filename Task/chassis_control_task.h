#ifndef __CHASSIS_CONTROL_TASK_H
#define __CHASSIS_CONTROL_TASK_H

#include "Struct_encapsulation.h"
#include "math.h"
#include "PID.h"
#include "delay.h"
#include "imu_control.h"

void Chassis_Control_Task_Create(void);
void Chassis_SetRelativeZero(void);
void Chassis_SetRelativeTarget(float dx, float dy, float dw);



#endif
