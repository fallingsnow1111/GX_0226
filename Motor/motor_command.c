#include "motor_command.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"

// 电机相关命令帧定义
#define MOTOR_FRAME_POS_LEN         8
#define MOTOR_FRAME_ACK_LEN         4
#define MOTOR_CMD_READ_POS          0x36
#define MOTOR_CMD_SPEED_CTRL        0xF6
#define MOTOR_FRAME_END             0x6B

#define MOTOR_POSITION_FRAME_LEN    13
#define MOTOR_POSITION_MODE_REL_LAST_CMD 0x00
#define MOTOR_POSITION_MODE_ABS_ZERO     0x01
#define MOTOR_POSITION_MODE_REL_CURRENT  0x02
#define MOTOR_POSITION_SPEED_RPM         600
#define MOTOR_POSITION_ACC               120
#define UART3_TX_TIMEOUT_MS              20U

#define MOTOR1_GAIN 1.00f
#define MOTOR2_GAIN 1.00f
#define MOTOR3_GAIN 0.95f
#define MOTOR4_GAIN 1.00f

#define magic_number 14  // some magic number for position conversion

volatile struct CHECK_FLAG_t motor_check;

struct MOTOR_DATA motor1;
struct MOTOR_DATA motor2;
struct MOTOR_DATA motor3;
struct MOTOR_DATA motor4;

static uint8_t LB_send[15];
static uint8_t LF_send[15];
static uint8_t RB_send[15];
static uint8_t RF_send[15];

static uint8_t rx_cache[RXdat_maxsize * 2];
static uint16_t rx_cache_len = 0;

static uint8_t Motor_IsValidID(uint8_t id) {
    return (id >= 1 && id <= 4);
}

// 消耗指定数量的字节，从接收缓存中移除已处理的数据
static void Motor_ConsumeBytes(uint16_t count)
{
    if (count >= rx_cache_len) {
        rx_cache_len = 0; // Clear cache if all bytes are consumed
        return;
    } 

    memmove(rx_cache, rx_cache + count, rx_cache_len - count);
    rx_cache_len -= count;
}

// 解析接收到的数据帧，提取电机位置信息
static void Motor_ParsePositionFrame(const uint8_t* frame)
{
    float angle;

    angle = (float)(((uint32_t)frame[3] << 24) | 
                    ((uint32_t)frame[4] << 16) | 
                    ((uint32_t)frame[5] << 8)  | 
                    ((uint32_t)frame[6]));

    angle = angle * 360.0f / 65535.0f; // Convert to degrees

    if (frame[2] == 0x01) {
        angle = -angle; // Negative direction
    }

    switch (frame[0]) {
        case 0x01:
            motor1.actual_angle = angle;
            motor_check.flag_finish |= (1 << 0); // Set bit 0
            break;
        case 0x02:
            motor2.actual_angle = angle;
            motor_check.flag_finish |= (1 << 1); // Set bit 1
            break;
        case 0x03:
            motor3.actual_angle = angle;
            motor_check.flag_finish |= (1 << 2); // Set bit 2
            break;
        case 0x04:
            motor4.actual_angle = angle;
            motor_check.flag_finish |= (1 << 3); // Set bit 3
            break;
        default:
            break; // Invalid ID, ignore
    }
}

// 解析速度控制命令的ACK帧，设置就绪标志
static void Motor_ParseAckFrame(const uint8_t* frame)
{
    if(Motor_IsValidID(frame[0])        &&
       frame[1] == MOTOR_CMD_SPEED_CTRL &&
       frame[2] == 0x02                 &&
       frame[3] == MOTOR_FRAME_END)
    {
        motor_check.flag_ready = finish; // Set ready flag
    }
}

// Reception buffer
static uint8_t RX_data[RXdat_maxsize]={0};

static HAL_StatusTypeDef Uart3_TxDma_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (huart3.gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }

        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        else
        {
            HAL_Delay(1);
        }
    }

    return HAL_OK;
}

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
    if (Uart3_TxDma_WaitReady(UART3_TX_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_UART_Transmit(&huart3, buf, len, UART3_TX_TIMEOUT_MS);
        return;
    }

    if (HAL_UART_Transmit_DMA(&huart3, buf, len) != HAL_OK)
    {
        (void)HAL_UART_Transmit(&huart3, buf, len, UART3_TX_TIMEOUT_MS);
        return;
    }

    if (Uart3_TxDma_WaitReady(UART3_TX_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_UART_Transmit(&huart3, buf, len, UART3_TX_TIMEOUT_MS);
    }
}

static void Motor_Send_Position_Sync(uint8_t mode)
{
    Send_Position_together((int)motor1.target_angle,
                           (int)motor2.target_angle,
                           (int)motor3.target_angle,
                           (int)motor4.target_angle,
                           mode);

    uart3WriteBuf(LB_send, MOTOR_POSITION_FRAME_LEN);
    uart3WriteBuf(LF_send, MOTOR_POSITION_FRAME_LEN);
    uart3WriteBuf(RF_send, MOTOR_POSITION_FRAME_LEN);
    uart3WriteBuf(RB_send, MOTOR_POSITION_FRAME_LEN);

    // 所有轴参数下发后，再统一触发同步启动
    Send_motor_together();
}

void Motor_Init(void)
{
    motor1.target_angle = 0;
    motor1.actual_angle = 0;
    motor2.target_angle = 0;
    motor2.actual_angle = 0;
    motor3.target_angle = 0;
    motor3.actual_angle = 0;
    motor4.target_angle = 0;
    motor4.actual_angle = 0;
    // 开启DMA接收，检测到空闲则产生回调
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RX_data, RXdat_maxsize);
}

//输入4个电机速度，单位rpm，16位有符号整数，范围-32768~32767
void Motor_Send_Speed_together(float LB, float LF, float RF, float RB)
{
    static uint8_t* LB_speedptr = LB_send;
    static uint8_t* LF_speedptr = LF_send;
    static uint8_t* RB_speedptr = RB_send;
    static uint8_t* RF_speedptr = RF_send;
    uint8_t* temp[4] = {LB_speedptr, LF_speedptr, RF_speedptr, RB_speedptr};
    int tempspeed = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t var = i;
        switch (i + 1) {
            case 1:
                tempspeed = LB;
                break;
            case 2:
                tempspeed = LF;
                break;
            case 3:
                tempspeed = RF;
                break;
            case 4:
                tempspeed = RB;
                break;
            default:
                tempspeed = 0;
                break;
        }

        temp[i][0] = var + 1;   // Motor ID
        temp[i][1] = 0xF6;      // Command for speed setting

        if (tempspeed > 0)
        {
            temp[i][2] = 0x00;
            temp[i][3] = (tempspeed >> 8) & 0xFF;   // High byte
            temp[i][4] = (tempspeed & 0xFF);        // Low byte
        }
        else
        {
            tempspeed = -tempspeed;
            temp[i][2] = 0x01;
            temp[i][3] = (tempspeed >> 8) & 0xFF;   // High byte
            temp[i][4] = (tempspeed & 0xFF);        // Low byte
        }
        temp[i][5] = 0xC8;     // Acceleration
        temp[i][6] = 0x01;     // 多机同步标志，1表示缓存后统一触发
        temp[i][7] = 0x6B;     // End byte
    }
}

//多机同步运动指令
void Send_motor_together(void) {
    uint8_t data[4];
    data[0] = 0x00;
    data[1] = 0xFF;
    data[2] = 0x66;
    data[3] = 0x6B;
    uart3WriteBuf(data, 4);
}

// 读取电机实时位置
void Motor_read_coordination(uint8_t motor_id)
{
    uint8_t TXdata[3];
    TXdata[0] = motor_id;
    TXdata[1] = 0x36;
    TXdata[2] = 0x6B;
    uart3WriteBuf((uint8_t*)TXdata, 3);
}

//位置控制模式，传入脉冲数，32位有符号整数
void Send_Position_together(int LB, int LF, int RF, int RB, char mode)
{
    static uint8_t* LB_postionptr = LB_send;
    static uint8_t* LF_postionptr = LF_send;
    static uint8_t* RB_postionptr = RB_send;
    static uint8_t* RF_postionptr = RF_send;
    uint8_t* temp[4] = {LB_postionptr, LF_postionptr, RF_postionptr, RB_postionptr};
    int temppostion = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t var = i;
        switch (i + 1) {
            case 1:
                temppostion = LB;
                break;
            case 2:
                temppostion = LF;
                break;
            case 3:
                temppostion = RF;
                break;
            case 4:
                temppostion = RB;
                break;
            default:
                temppostion = 0;
                break;
        }
        temp[i][0] = var + 1;   // Motor ID
        temp[i][1] = 0xFD;      // Command for position setting
        if (temppostion > 0)
        {
            temp[i][2] = 0x00;  // Direction
        }
        else
        {
            temppostion = -temppostion;
            temp[i][2] = 0x01;
        }
        temp[i][3] = (uint8_t)((MOTOR_POSITION_SPEED_RPM >> 8) & 0xFF); // Speed high byte
        temp[i][4] = (uint8_t)(MOTOR_POSITION_SPEED_RPM & 0xFF);        // Speed low byte

        temp[i][5] = MOTOR_POSITION_ACC; // ACC

        // Pulse position (4 bytes)
        temp[i][6] = (uint8_t)((temppostion * magic_number) >> 24);
        temp[i][7] = (uint8_t)((temppostion * magic_number) >> 16);
        temp[i][8] = (uint8_t)((temppostion * magic_number) >> 8);
        temp[i][9] = (uint8_t)(temppostion *  magic_number);

        temp[i][10] = (uint8_t)mode;    // Absolute/Relative mode
        temp[i][11] = 0x01;             // 多机同步标志，1表示缓存后统一触发
        temp[i][12] = 0x6B;             // End byte
    }
}

void Send_speed_switch(void)
{
    uart3WriteBuf(LB_send,8);
    uart3WriteBuf(LF_send,8);
    uart3WriteBuf(RF_send,8);
    uart3WriteBuf(RB_send,8);

    // 四轮速度参数都缓存后，统一触发同步启动
    Send_motor_together();
}

// 左手坐标系运动解算，逆时针为正
void Motor_Action_Calculate_target(float vx, float vy, float vw) {
    taskENTER_CRITICAL();
    motor1.target_angle = (vw + vy + vx) * MOTOR1_GAIN; // 1号电机
    motor2.target_angle = (vw + vy - vx) * MOTOR2_GAIN; // 2号电机
    motor3.target_angle = (vw - vy - vx) * MOTOR3_GAIN; // 3号电机
    motor4.target_angle = (vw - vy + vx) * MOTOR4_GAIN; // 4号电机
    taskEXIT_CRITICAL();
}

// 位置解算，计算实际位置
void Motor_Action_Calculate_actual(volatile float *actual_x, volatile float *actual_y, volatile float *actual_w) {
    taskENTER_CRITICAL();
    *actual_x = (motor1.actual_angle - motor2.actual_angle - motor3.actual_angle + motor4.actual_angle) / 4.0f;
    *actual_y = (motor1.actual_angle + motor2.actual_angle - motor3.actual_angle - motor4.actual_angle) / 4.0f;
    *actual_w = (motor1.actual_angle + motor2.actual_angle + motor3.actual_angle + motor4.actual_angle) / 4.0f;
    taskEXIT_CRITICAL();
}

// 发送四轮速度并同步触发执行
void Motor_setspeed(float vx, float vy, float vw)
{
    Motor_Action_Calculate_target(vx, vy, vw);
    Motor_Send_Speed_together(motor1.target_angle, motor2.target_angle, motor3.target_angle, motor4.target_angle);
    Send_speed_switch();
}

void Motor_setposition_relative(float dx, float dy, float dw)
{
    Motor_Action_Calculate_target(dx, dy, dw);
    Motor_Send_Position_Sync(MOTOR_POSITION_MODE_REL_CURRENT);
}

void Motor_setposition_absolute_zero(float x, float y, float w)
{
    Motor_Action_Calculate_target(x, y, w);
    Motor_Send_Position_Sync(MOTOR_POSITION_MODE_ABS_ZERO);
}

// 将当前位置角度清零
void Motor_SetZero(void)
{
    uint8_t TXdata[4];
    TXdata[1] = 0x0A;
    TXdata[2] = 0x6D;
    TXdata[3] = 0x6B;
    for (uint16_t i = 0x01; i <= 4; i++)
    {
        TXdata[0] = i;
        uart3WriteBuf((uint8_t*)TXdata, 4);
        Delay_ms(10);
    }
}

void USART3_Process_data(uint8_t* data, uint8_t len)
{
    uint8_t expected_len = 0;

    if(len == 0)
    {
        return;
    }

    if((uint16_t)(rx_cache_len + len) > sizeof(rx_cache))
    {
        // 如果新数据超过缓存大小，丢弃旧数据
        rx_cache_len = 0;
    }

    memcpy(rx_cache + rx_cache_len, data, len);
    rx_cache_len += len;

    while(rx_cache_len > 0)
    {
        if(!Motor_IsValidID(rx_cache[0]))
        {
            // 无效ID，丢弃第一个字节
            Motor_ConsumeBytes(1);
            continue;
        }
        if(rx_cache_len < 2)
        {
            break; // 等待更多数据
        }
        if(rx_cache[1] == MOTOR_CMD_READ_POS)
        {
            expected_len = MOTOR_FRAME_POS_LEN;
        }
        else if(rx_cache[1] == MOTOR_CMD_SPEED_CTRL)
        {
            expected_len = MOTOR_FRAME_ACK_LEN;
        }
        else
        {
            // 无效命令，丢弃第一个字节
            Motor_ConsumeBytes(1);
            continue;
        }

        if(rx_cache_len < expected_len)
        {
            break; // 等待更多数据
        }

        if(rx_cache[expected_len - 1] != MOTOR_FRAME_END)
        {
            // 无效帧，丢弃第一个字节
            Motor_ConsumeBytes(1);
            continue;
        }

        if(expected_len == MOTOR_FRAME_POS_LEN)
        {
            Motor_ParsePositionFrame(rx_cache);
        }
        else
        {
            Motor_ParseAckFrame(rx_cache);
        }

        Motor_ConsumeBytes(expected_len); // 移除已处理的帧
    }
}

void My_UART3_IRQHandler(uint16_t Size)
{
    if(Size > 0 && Size <= RXdat_maxsize) {
        USART3_Process_data(RX_data, (uint8_t)Size);
    }
    //处理完数据重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RX_data, RXdat_maxsize);
}














