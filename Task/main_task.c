#include "main_task.h"

TaskHandle_t Main_Task_Handle;

#define MAIN_TASK_STACK_SIZE 512
#define MAIN_TASK_PRIORITY 7

extern volatile uint8_t MOTOR_ACTION_FINISH_FLAG;

// 等待底盘动作完成的函数
static void Wait_Chassis_Finish(void)
{
    while (MOTOR_ACTION_FINISH_FLAG != finish) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms后再次检查
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // 确保动作完全停止后再继续
}

void Main_Task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(300)); // 等待系统稳定

    Chassis_SetRelativeZero(); // 设置当前位姿为分段零点

    while (1)
    {
        // 前进
        Chassis_SetRelativeTarget(100.0f, 0.0f, 0.0f);
        Wait_Chassis_Finish();

        vTaskDelay(pdMS_TO_TICKS(200));

        // 后退
        Chassis_SetRelativeTarget(-100.0f, 0.0f, 0.0f);
        Wait_Chassis_Finish();

        vTaskDelay(pdMS_TO_TICKS(200));

        // 左移
        Chassis_SetRelativeTarget(0.0f, 100.0f, 0.0f);
        Wait_Chassis_Finish();

        vTaskDelay(pdMS_TO_TICKS(200));

        // 右移
        Chassis_SetRelativeTarget(0.0f, -100.0f, 0.0f);
        Wait_Chassis_Finish();

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 主任务的循环逻辑
        vTaskDelay(pdMS_TO_TICKS(100)); // 这里设置为100ms周期，可以根据需要调整
    }
}

void Main_Task_Create(void)
{
    xTaskCreate(Main_Task,
                "Main_Task",
                MAIN_TASK_STACK_SIZE,
                NULL,
                MAIN_TASK_PRIORITY,
                &Main_Task_Handle);
}

