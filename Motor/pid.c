#include "pid.h"

//两数差的绝对值
float _ABS(float a, float b)
{
    return (a > b) ? (a - b) : (b - a);
}

void PID_init(struct PID_struct *pid, float kp, float ki, float kd, float outputmax, float outputmin)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integralMax = 100;
    pid->outputMax = outputmax;
    pid->outputMin = outputmin;
    pid->integral = 0;
    pid->previousError = 0;
}

//位置式PID
float PID_Compute(struct PID_struct *pid, float target_value, float current_value)
{
    float error = target_value - current_value;
    pid->integral += error;
    if (pid->integral > pid->integralMax)
    {
        pid->integral = pid->integralMax;
    }
    else if (pid->integral < -pid->integralMax)
    {
        pid->integral = -pid->integralMax;
    }
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd *  (error - pid->previousError);
    pid->previousError = error;
    if (output > pid->outputMax)
    {
         output = pid->outputMax;
    }
    else if (output < pid->outputMin)
    {
        output = pid->outputMin;
    }
    return output;
}



