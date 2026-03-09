#include "imu_control.h"

#include <math.h>
#include <wchar.h>

#include "motor_control.h"

struct PID_struct Gyro_PID;
struct IMU_RUNDATA imu_run;
struct IMU_RUNDATA imu_turn;

void Gyro_Init(void)
{
    PID_init(&Gyro_PID, 2.6, 0, 0,150, -150);
    IMU_Receive_Init();
}

float getAngleZ(float yaw, float my_angle)
{
    float temp;
    temp = (yaw - my_angle);
    if (yaw != my_angle)
    {
        if (temp < -180)
        {
            temp += 360;
        }
        else if (temp > 180)
        {
            temp -= 360;
        }
    }
    return temp;
}

float getAngleZ_avg(float my_angle)
{
    float temp;
    temp = getAngleZ(imu.yaw, my_angle);
    return temp;
}

//角度修偏函数，解决角度在正负180度之间的跳变
float normalizeAngle(float angle)
{
    static int16_t turn_count = 0;
    float delta;

    if(imu_run.IS_MOVING == 0)
    {
        turn_count = 0; //静止时重置转圈计数
        imu_run.IS_MOVING = 1;
        imu_run.LAST_ANGLE = angle;
        return angle;
    }

    delta = angle - imu_run.LAST_ANGLE;
    if (fabsf(delta) > 180.0f)
    {
        if (delta > 0)
        {
            turn_count--;
        }
        else
        {
            turn_count++;
        }
    }

    imu_run.LAST_ANGLE = angle;
    return angle + 360.0f * turn_count;
}

//反向控制的原因是？
void Direction_Calibration(float target_angle)
{
    float current_angle = normalizeAngle(imu.yaw);
    float w_output = 0;
    if (_ABS(target_angle, current_angle) > 0.6f)
    {
        w_output = PID_Compute(&Gyro_PID, target_angle, current_angle);
        Motor_setspeed(0,0,-w_output);
    }
    else
    {
        Motor_setspeed(0,0,0);
        HAL_Delay(6);
    }
}

float Direction_Calibration_turn(float target_angle)
{
    float current_angle = normalizeAngle(imu.yaw);
    float w_output = PID_Compute(&Gyro_PID, target_angle, current_angle);
    // 小输出时设置死区（防止电机抖动）
    if (fabsf(w_output) < 2.0)
    {
        w_output = (w_output > 0) ? 1.0f : -1.0f;
    }
    return -w_output;
}





