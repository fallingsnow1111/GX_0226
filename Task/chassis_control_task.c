#include "chassis_control_task.h"

#define CHASSIS_CONTROL_TASK_H_STACK 512//任务堆栈
#define CHASSIS_CONTROL_TASK_H_PRIORITY 6//优先级

#define POSITION_THRESHOLD 15.0f //位置误差阈值
#define ORIENTATION_THRESHOLD 1.0f //姿态误差阈值
#define MIN_TRANSLATION_OUTPUT 8.0f //最小平移输出
#define ALL_MOTOR_FEEDBACK_READY 0x0F //所有电机反馈就绪标志

TaskHandle_t Chassis_Control_Task_Handle;

extern volatile struct CHECK_FLAG_t motor_check;

volatile CARDATA_T car;
static struct PID_struct chassis_pid_x;
static struct PID_struct chassis_pid_y;

static float x_c_output = 0.0f; //X轴PID输出
static float y_c_output = 0.0f; //Y轴PID输出
static float w_c_output = 0.0f; //角度PID输出

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

// 死区处理函数
static float apply_deadzone(float output,
                            float error,
                            float threshold,
                            float min_output) 
{
    if (fabsf(error) <= threshold) {
        return 0.0f;
    }
    if(output > 0.0f && output < min_output) {
        return min_output;
    } 
    else if(output < 0.0f && output > -min_output) {
        return -min_output;
    }
    return output;
}

void Get_Chassis_c_output(float *x_output, float *y_output, float *w_output)
{
    *x_output = x_c_output;
    *y_output = y_c_output;
    *w_output = w_c_output;
}

void chassis_control_init(void)
{
    PID_init(&chassis_pid_y, 0.28f, 0.0f, 0.0f, 350.0f, -350.0f);
    PID_init(&chassis_pid_x, 0.28f, 0.0f, 0.0f, 350.0f, -350.0f);

    car.target_x = 0.0f;
    car.target_y = 0.0f;
    car.target_w = 0.0f;

    car.actual_x = 0.0f;
    car.actual_y = 0.0f;
    car.actual_w = 0.0f;

    car.IMU_modeable = unable;
    car.ODOM_modeable = enable;
}

// 里程计归零函数
void Odometer_SetZero(void)
{
    car.actual_x = 0.0f;
    car.actual_y = 0.0f;
    car.actual_w = 0.0f;
}

void chassis_control(void)
{
    float err_x;
    float err_y;
    float err_w;
    static uint8_t lost_feedback_count = 0;//丢失反馈计数器
    uint8_t feedback_ready = motor_check.flag_finish;

    if(feedback_ready == ALL_MOTOR_FEEDBACK_READY)
    {
        lost_feedback_count = 0; //重置丢失反馈计数器

         Motor_Action_Calculate_actual(&car.actual_x, &car.actual_y, &car.actual_w);
        //如果IMU模式使能，使用IMU角度修正实际角度
        if(car.IMU_modeable == enable)
        {
            car.actual_w = normalizeAngle(imu.yaw);
        }

        //计算位置和姿态误差
        err_x = car.target_x - car.actual_x;
        err_y = car.target_y - car.actual_y;
        err_w = angle_error_deg(car.target_w, car.actual_w);

        //计算PID输出
        x_c_output = PID_Compute(&chassis_pid_x, car.target_x, car.actual_x);
        y_c_output = PID_Compute(&chassis_pid_y, car.target_y, car.actual_y);   
        w_c_output = Direction_Calibration_turn(car.target_w);

        //应用死区处理，防止小误差导致电机抖动
        y_c_output = apply_deadzone(y_c_output, err_y, POSITION_THRESHOLD, MIN_TRANSLATION_OUTPUT);
        x_c_output = apply_deadzone(x_c_output, err_x, POSITION_THRESHOLD, MIN_TRANSLATION_OUTPUT);

        if(fabsf(err_w) <= ORIENTATION_THRESHOLD) {
            w_c_output = 0.0f; //如果角度误差在阈值内，停止旋转
        }

        if(fabsf(err_x) <= POSITION_THRESHOLD
        && fabsf(err_y) <= POSITION_THRESHOLD
        && fabsf(err_w) <= ORIENTATION_THRESHOLD)
        {
            Motor_setspeed(0.0f, 0.0f, 0.0f);

            if(MOTOR_ACTION_FINISH_FLAG == Incomplete) {
                MOTOR_ACTION_FINISH_FLAG = finish; //设置动作完成标志
                Motor_SetZero(); //动作完成后将电机位置归零
            }
        }
        else
        {
            MOTOR_ACTION_FINISH_FLAG = Incomplete; //动作未完成
            Motor_setspeed(x_c_output, y_c_output, w_c_output);
        }
    }
    else
    {
        MOTOR_ACTION_FINISH_FLAG = Incomplete; //动作未完成

        if(++lost_feedback_count >= 5) {
            //如果连续多次丢失反馈，停止电机以防止失控
            Motor_setspeed(0.0f, 0.0f, 0.0f);
        }
    }

    motor_check.flag_finish = 0; //重置反馈完成标志，等待下一轮反馈
    motor_read_coordination_all();
}

void Chassis_Control_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms周期

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
