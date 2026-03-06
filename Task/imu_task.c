#include "imu_task.h"
#include "task.h"

TaskHandle_t imu_task_handle;

#define IMU_TASK_STACK_SIZE 256
#define IMU_TASK_PRIORITY 6

void IMU_Task(void *pvParameters)
{
    while(1)
    {
        vTaskDelay(1000);
    }
}

void IMU_Task_Create(void)
{
    xTaskCreate(IMU_Task, "IMU_Task", IMU_TASK_STACK_SIZE, NULL, IMU_TASK_PRIORITY, &imu_task_handle );
}



