#ifndef __IMU_H
#define __IMU_H

//uint8_t定义在此
#include <stdint.h>
#include "usart.h"

//全局变量外部声明，供其他源文件使用
extern struct IMU imu;
//引用其他源文件定义的变量
extern DMA_HandleTypeDef hdma_usart2_rx;
extern struct IMU_RUNDATA imu_run;

typedef struct IMU
{
    float roll;     //x
    float pitch;    //y
    float yaw;      //z
};

void IMU_Receive_Init(void);
void IMU_Process(void);
void IMU_SetZero(void);
void Imu_unlock_register(void);
void Imu_setset_baudrate_115200(void);
void Imu_setsave_settings(void);
void Imu_set200hz(void);

#endif

