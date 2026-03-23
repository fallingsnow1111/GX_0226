#include "main_task.h"

TaskHandle_t Main_Task_Handle;

#define MAIN_TASK_STACK_SIZE 512
#define MAIN_TASK_PRIORITY 5

#define MAIN_TASK_ACTION_SETTLE_MS 50
#define MAIN_TASK_WAIT_TIMEOUT_MS 5000
#define MAIN_TASK_PRECHECK_DELAY_MS 60
#define MAIN_TASK_STEP_GAP_MS 50

extern volatile uint8_t MOTOR_ACTION_FINISH_FLAG;

// 带超时等待，避免主任务卡死在等待完成标志
static BaseType_t Wait_Chassis_Finish(TickType_t timeout_ticks)
{
    TickType_t start_tick = xTaskGetTickCount();

    while (MOTOR_ACTION_FINISH_FLAG != finish) {
        if ((xTaskGetTickCount() - start_tick) >= timeout_ticks) {
            return pdFALSE;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms后再次检查
    }

    vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_ACTION_SETTLE_MS)); // 确保动作完全停止后再继续
    return pdTRUE;
}

static void Main_Task_RunStep(float dx, float dy, float dw)
{
    Chassis_SetRelativeTarget(dx, dy, dw);

    // 先给控制任务一点时间消化新目标，避免刚下发就立即判断
    vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_PRECHECK_DELAY_MS));

    if (Wait_Chassis_Finish(pdMS_TO_TICKS(MAIN_TASK_WAIT_TIMEOUT_MS)) == pdFALSE) {
        Motor_setspeed(0.0f, 0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_STEP_GAP_MS));
    }
}

void Main_Task(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(300)); // 等待系统稳定

    Chassis_SetRelativeZero(); // 设置当前位姿为分段零点
	
	vTaskDelay(1000);

    while (1)
    {
        // 前进
        Main_Task_RunStep(500.0f, 0.0f, 0.0f);

        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_STEP_GAP_MS));

        // 后退
        Main_Task_RunStep(-500.0f, 0.0f, 0.0f);

        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_STEP_GAP_MS));

//        // 左移
//        Main_Task_RunStep(0.0f, 500.0f, 0.0f);

//        vTaskDelay(pdMS_TO_TICKS(200));

//        // 右移
//        Main_Task_RunStep(0.0f, -500.0f, 0.0f);

        vTaskDelay(pdMS_TO_TICKS(1000));

        // 所有动作都已带等待与超时保护，这里不再追加额外循环延时
    }
}

void Main_Task_Create(void)
{
    if (xTaskCreate(Main_Task,
                    "Main_Task",
                    MAIN_TASK_STACK_SIZE,
                    NULL,
                    MAIN_TASK_PRIORITY,
                    &Main_Task_Handle) != pdPASS)
    {
        Error_Handler();
    }
}

