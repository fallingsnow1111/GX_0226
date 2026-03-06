#ifndef __PID_H
#define __PID_H

#include "Struct_encapsulation.h"

float _ABS(float a, float b);
void PID_init(struct PID_struct *pid, float kp, float ki, float kd, float outputmax, float outputmin);
float PID_Compute(struct PID_struct *pid, float target_value, float current_value);

#endif
