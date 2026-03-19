#include "imu_task.h"

TaskHandle_t IMU_Task_Handle;

#define IMU_TASK_STACK_SIZE 256
#define IMU_TASK_PRIORITY 5
#define IMU_TASK_PERIOD_MS 5

void IMU_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);

    while (1)
    {
        IMU_Process();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void IMU_Task_Create(void)
{
    xTaskCreate(IMU_Task,
                "IMU_Task",
                IMU_TASK_STACK_SIZE,
                NULL,
                IMU_TASK_PRIORITY,
                &IMU_Task_Handle);
}
