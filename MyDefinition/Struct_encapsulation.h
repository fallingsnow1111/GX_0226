#ifndef __STRUCT_ENCAPSULATION_H
#define __STRUCT_ENCAPSULATION_H

#include "stdint.h"

#define finish      1
#define Incomplete  0

typedef enum {
    enable=1,
    unable=0
}ABLE_T;

typedef struct MOTOR_SPEED_t
{
    float x_setpeed;  // X轴速度
    float y_setpeed;  // Y轴速度
    float w_setpeed;  // 角速度
}MOTOR_SPEED_t;

typedef struct CARDATA_T {
    float target_y;  // 目标里程计Y
    float target_x;  // 目标里程计X
    float actual_y;  // 实际里程计Y
    float actual_x;  // 实际里程计X
    float target_w;  // 目标里程计角度
    float actual_w;  // 实际里程计角度
    volatile ABLE_T IMU_modeable; // IMU模式使能标志
    volatile ABLE_T ODOM_modeable; // 里程计模式使能标志
    volatile ABLE_T action_modeable; // 动作模式使能标志
}CARDATA_T;

typedef struct CHECK_FLAG_t {
    uint8_t flag_ready;     //任务是否就绪的标识位
    uint8_t flag_finish;    //任务是否完成的标志位
};

typedef struct PID_struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float integralMax;
    float  outputMin;
    float  outputMax;
    float previousError;
};

#endif

