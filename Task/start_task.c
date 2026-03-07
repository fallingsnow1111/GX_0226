#include "start_task.h"

TaskHandle_t Start_Task_Handle;
TaskHandle_t Init_Task_Handle;

#define START_TASK_STACK_SIZE 512
#define INIT_TASK_STACK_SIZE 256
#define START_TASK_PRIORITY 5
#define INIT_TASK_PRIORITY 6

void Init_Task(void *pvParameters)
{
    Delay_Init();
    Gyro_Init();
    // Motor_Init();

    // IMU_Task_Create(); // 创建 IMU 任务

    while (1)
    {
        // Motor_setspeed(0,0,10);
        vTaskDelay(200);
    }
}

void Init_Task_Create(void)
{
    xTaskCreate(Init_Task,"Init_Task",INIT_TASK_STACK_SIZE,NULL,INIT_TASK_PRIORITY,&Init_Task_Handle);
}

void Start_Task(void *pvParameters)
{
    taskENTER_CRITICAL();

    Init_Task_Create();

    taskEXIT_CRITICAL();

    vTaskDelete(NULL);
}

void Start_Task_Create(void)
{
    xTaskCreate(Start_Task,"Start_task",START_TASK_STACK_SIZE,NULL,START_TASK_PRIORITY,&Start_Task_Handle);
}




