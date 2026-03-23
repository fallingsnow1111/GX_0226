#ifndef __CHASSIS_CONTROL_TASK_H
#define __CHASSIS_CONTROL_TASK_H

#include "cmsis_os.h"

#include "main.h"
#include "math.h"

void chassis_control_init(void);
void Chassis_Control_Task_Create(void);
void Chassis_SetRelativeZero(void);
void Chassis_SetRelativeTarget(float dx, float dy, float dw);

#endif
