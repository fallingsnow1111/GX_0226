#include "chassis_control_task.h"
#include "PID.h"

#define CHASSIS_CONTROL_TASK_H_STACK 512//任务堆栈
#define CHASSIS_CONTROL_TASK_H_PRIORITY 6//优先级

#define POSITION_THRESHOLD 5.0f //位置误差阈值
#define ORIENTATION_THRESHOLD 1.0f //姿态误差阈值
#define MIN_TRANSLATION_OUTPUT 3.0f //最小平移输出
#define NEAR_TARGET_WINDOW 100.0f //近目标减速窗口
#define HEADING_HOLD_GAIN 0.8f //平移时角度锁定补偿
#define PLAN_BRAKE_ACC_XY 5.0f //平移刹车规划加速度，用于sqrt(2*a*|err|)
#define PLAN_BRAKE_ACC_W  8.0f //旋转刹车规划加速度，用于sqrt(2*a*|err|)
#define PLAN_MAX_DV_XY_PER_CYCLE 30.0f //20ms周期下每个控制周期的平移最大速度变化
#define PLAN_MAX_DW_PER_CYCLE 15.0f //20ms周期下每个控制周期的角速度最大变化
#define ALL_MOTOR_FEEDBACK_READY 0x0F //所有电机反馈就绪标志
#define FINISH_STABLE_COUNT 3 //20ms周期下约60ms稳定判定
#define LOST_FEEDBACK_STOP_COUNT 5 //20ms周期下约100ms失联停车

TaskHandle_t Chassis_Control_Task_Handle;

extern volatile struct CHECK_FLAG_t motor_check;
volatile uint8_t MOTOR_ACTION_FINISH_FLAG = Incomplete;//电机动作完成标志

static float segment_zero_x = 0.0f; //分段归零的X坐标
static float segment_zero_y = 0.0f; //分段归零的Y坐标
static float segment_zero_w = 0.0f; //分段归零的角度

volatile CARDATA_T car;
static struct PID_struct chassis_pid_x;
static struct PID_struct chassis_pid_y;
static struct PID_struct chassis_pid_w;

static float x_c_output = 0.0f; //X轴速度输出
static float y_c_output = 0.0f; //Y轴速度输出
static float w_c_output = 0.0f; //角速度输出
static float x_plan_output = 0.0f; //规划后的X轴速度输出
static float y_plan_output = 0.0f; //规划后的Y轴速度输出
static float w_plan_output = 0.0f; //规划后的角速度输出

// 角度误差计算函数
static float angle_error_deg(float target_angle, float actual_angle) {
    float error = target_angle - actual_angle;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}

static float apply_deadzone(float output,
                            float error,
                            float threshold,
                            float min_output)
{
    if (fabsf(error) <= threshold)
    {
        return 0.0f;
    }

    if (output > 0.0f && output < min_output)
    {
        return min_output;
    }

    if (output < 0.0f && output > -min_output)
    {
        return -min_output;
    }

    return output;
}

static float limit_delta(float target, float current, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    return current + delta;
}

static float clamp_by_abs_limit(float value, float abs_limit)
{
    if (value > abs_limit)
    {
        return abs_limit;
    }

    if (value < -abs_limit)
    {
        return -abs_limit;
    }

    return value;
}

static uint8_t Chassis_IsFinishedByError(float err_x, float err_y, float err_w)
{
    return (fabsf(err_x) <= POSITION_THRESHOLD
         && fabsf(err_y) <= POSITION_THRESHOLD
         && fabsf(err_w) <= ORIENTATION_THRESHOLD) ? 1U : 0U;
}

void Get_Chassis_c_output(float *x_output, float *y_output, float *w_output)
{
    *x_output = x_plan_output;
    *y_output = y_plan_output;
    *w_output = w_plan_output;
}

// 里程计归零函数
void Odometer_SetZero(void)
{
    car.actual_x = 0.0f;
    car.actual_y = 0.0f;
    car.actual_w = 0.0f;
}

// 分段归零函数，适用于需要在运动过程中重新设定参考点的情况
void Chassis_SetRelativeZero(void)
{
    segment_zero_x = car.actual_x;
    segment_zero_y = car.actual_y;
    segment_zero_w = car.actual_w;
}

// 启动相对运动，输入相对于当前分段零点的目标位置和姿态
void Chassis_SetRelativeTarget(float dx, float dy, float dw)
{
    segment_zero_x = car.actual_x;
    segment_zero_y = car.actual_y;
    segment_zero_w = car.actual_w;

    car.target_x = dx;
    car.target_y = dy;
    car.target_w = dw;

    MOTOR_ACTION_FINISH_FLAG = Incomplete; //设置动作未完成标志
}

void chassis_control_init(void)
{
    PID_init(&chassis_pid_x, 0.28f, 0.0f, 0.0f, 350.0f, -350.0f);
    PID_init(&chassis_pid_y, 0.28f, 0.0f, 0.0f, 350.0f, -350.0f);
    PID_init(&chassis_pid_w, 2.0f, 0.0f, 0.0f, 180.0f, -180.0f);

    car.target_x = 0.0f;
    car.target_y = 0.0f;
    car.target_w = 0.0f;

    car.actual_x = 0.0f;
    car.actual_y = 0.0f;
    car.actual_w = 0.0f;

    segment_zero_x = 0.0f;
    segment_zero_y = 0.0f;
    segment_zero_w = 0.0f;

    x_plan_output = 0.0f;
    y_plan_output = 0.0f;
    w_plan_output = 0.0f;

    car.IMU_modeable = enable;
    car.ODOM_modeable = enable;
}


void chassis_control(void)
{
    float err_x;
    float err_y;
    float err_w;
    float rel_actual_x;
    float rel_actual_y;
    float rel_actual_w;
    float x_min_output;
    float y_min_output;
    static uint8_t finish_stable_count = 0;

    static uint8_t lost_feedback_count = 0;//丢失反馈计数器
    uint8_t feedback_ready;

    taskENTER_CRITICAL();
    feedback_ready = motor_check.flag_finish;
    motor_check.flag_finish = 0; // 快照后立即清零，避免与中断置位竞态
    taskEXIT_CRITICAL();

    if(feedback_ready == ALL_MOTOR_FEEDBACK_READY)
    {
        lost_feedback_count = 0; //重置丢失反馈计数器

         Motor_Action_Calculate_actual(&car.actual_x, &car.actual_y, &car.actual_w);
        //如果IMU模式使能，使用IMU角度修正实际角度
        if(car.IMU_modeable == enable)
        {
            car.actual_w = normalizeAngle(imu.yaw);
        }

        //计算相对于分段零点的实际位置和姿态
        rel_actual_x = car.actual_x - segment_zero_x;
        rel_actual_y = car.actual_y - segment_zero_y;
        rel_actual_w = angle_error_deg(car.actual_w, segment_zero_w);

        //计算位置和姿态误差
        err_x = car.target_x - rel_actual_x;
        err_y = car.target_y - rel_actual_y;
        err_w = angle_error_deg(car.target_w, rel_actual_w);

        x_c_output = PID_Compute(&chassis_pid_x, car.target_x, rel_actual_x);
        y_c_output = PID_Compute(&chassis_pid_y, car.target_y, rel_actual_y);
        w_c_output = PID_Compute(&chassis_pid_w, car.target_w, rel_actual_w);

        if (fabsf(car.target_w) < 1e-3f)
        {
            w_c_output += -HEADING_HOLD_GAIN * rel_actual_w;
        }

        // 基于剩余距离约束速度上限：v <= sqrt(2*a*|err|)
        x_c_output = clamp_by_abs_limit(x_c_output, sqrtf(2.0f * PLAN_BRAKE_ACC_XY * fabsf(err_x)));
        y_c_output = clamp_by_abs_limit(y_c_output, sqrtf(2.0f * PLAN_BRAKE_ACC_XY * fabsf(err_y)));
        w_c_output = clamp_by_abs_limit(w_c_output, sqrtf(2.0f * PLAN_BRAKE_ACC_W * fabsf(err_w)));

        x_min_output = (fabsf(err_x) < NEAR_TARGET_WINDOW) ? 0.0f : MIN_TRANSLATION_OUTPUT;
        y_min_output = (fabsf(err_y) < NEAR_TARGET_WINDOW) ? 0.0f : MIN_TRANSLATION_OUTPUT;

        x_c_output = apply_deadzone(x_c_output, err_x, POSITION_THRESHOLD, x_min_output);
        y_c_output = apply_deadzone(y_c_output, err_y, POSITION_THRESHOLD, y_min_output);
        if (fabsf(err_w) <= ORIENTATION_THRESHOLD)
        {
            w_c_output = 0.0f;
        }

        if(Chassis_IsFinishedByError(err_x, err_y, err_w))
        {
            x_plan_output = 0.0f;
            y_plan_output = 0.0f;
            w_plan_output = 0.0f;
            Motor_setspeed(0.0f, 0.0f, 0.0f);

            if (finish_stable_count < FINISH_STABLE_COUNT)
            {
                finish_stable_count++;
            }

            if (finish_stable_count >= FINISH_STABLE_COUNT && MOTOR_ACTION_FINISH_FLAG == Incomplete)
            {
                MOTOR_ACTION_FINISH_FLAG = finish; //设置动作完成标志
            }
        }
        else
        {
            finish_stable_count = 0;
            MOTOR_ACTION_FINISH_FLAG = Incomplete; //动作未完成

            x_plan_output = limit_delta(x_c_output, x_plan_output, PLAN_MAX_DV_XY_PER_CYCLE);
            y_plan_output = limit_delta(y_c_output, y_plan_output, PLAN_MAX_DV_XY_PER_CYCLE);
            w_plan_output = limit_delta(w_c_output, w_plan_output, PLAN_MAX_DW_PER_CYCLE);

            Motor_setspeed(x_plan_output, y_plan_output, w_plan_output);
        }
    }
    else
    {
        if(MOTOR_ACTION_FINISH_FLAG != finish) 
        {
            MOTOR_ACTION_FINISH_FLAG = Incomplete; //如果之前未完成，继续保持未完成状态
        }

        if(++lost_feedback_count >= LOST_FEEDBACK_STOP_COUNT) {
            //如果连续多次丢失反馈，停止电机以防止失控
            x_plan_output = 0.0f;
            y_plan_output = 0.0f;
            w_plan_output = 0.0f;
            Motor_setspeed(0.0f, 0.0f, 0.0f);
            finish_stable_count = 0;
        }
    }

    motor_read_coordination_all();
}

void Chassis_Control_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms周期

    while (1)
    {
        chassis_control();
        vTaskDelayUntil(&xLastWakeTime, xFrequency); //保持周期执行
    }
}

void Chassis_Control_Task_Create(void)
{
    xTaskCreate(Chassis_Control_Task, "Chassis_Control_Task", CHASSIS_CONTROL_TASK_H_STACK, NULL, CHASSIS_CONTROL_TASK_H_PRIORITY, &Chassis_Control_Task_Handle);
}
