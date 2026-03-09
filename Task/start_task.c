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
//    Gyro_Init();
    Motor_Init();
    chassis_control_init();

    motor_check.flag_finish = 0; //初始化电机反馈完成标志
    motor_read_coordination_all(); //读取初始电机位置

    Chassis_Control_Task_Create();
    Main_Task_Create();
    
    vTaskDelete(NULL);
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




