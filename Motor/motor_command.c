#include "motor_command.h"

#include <string.h>

#include "delay.h"

//step angle division 1.8/16=0.11255 degree
// #define Angle_division 16
#define magic_number 14  // some magic number for position conversion

MOTOR_SPEED_t car_setspeed;
volatile struct CHECK_FLAG_t motor_check;

struct MOTOR_DATA motor1;
struct MOTOR_DATA motor2;
struct MOTOR_DATA motor3;
struct MOTOR_DATA motor4;

static uint8_t LB_send[15];
static uint8_t LF_send[15];
static uint8_t RB_send[15];
static uint8_t RF_send[15];

// Reception buffer
static uint8_t RX_data[RXdat_maxsize]={0};
static char RX_piont=0;

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit_DMA(&huart3,buf,len);
}

void Motor_Init(void)
{
    car_setspeed.x_setpeed = 0.0f;
    car_setspeed.y_setpeed = 0.0f;
    car_setspeed.w_setpeed = 0.0f;
    motor1.target_angle = 0;
    motor1.actual_angle = 0;
    motor2.target_angle = 0;
    motor2.actual_angle = 0;
    motor3.target_angle = 0;
    motor3.actual_angle = 0;
    motor4.target_angle = 0;
    motor4.actual_angle = 0;
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE); // Enable USART3 interrupt
    //开启dma接收，检测到空闲则产生中断并停止
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RX_data, RXdat_maxsize); // Start DMA reception
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

        // if (__fbs(tempspeed) > 32767)
        // {
        //     //空逻辑？
        // }

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
        temp[i][5] = 0xC8;     //Acceleration
        temp[i][6] = 0x00;     // Multi-machine synchronization flag,0 for no sync
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
        temp[i][3] = 0x2E; // Speed high byte
        temp[i][4] = 0xE0; // Speed low byte

        temp[i][5] = 0xAE; // ACC original

        // Pulse position (4 bytes)
        temp[i][6] = (uint8_t)((temppostion * magic_number) >> 24);
        temp[i][7] = (uint8_t)((temppostion * magic_number) >> 16);
        temp[i][8] = (uint8_t)((temppostion * magic_number) >> 8);
        temp[i][9] = (uint8_t)(temppostion *  magic_number);

        temp[i][10] = (uint8_t)mode;    // Absolute/Relative mode
        temp[i][11] = 0x01;             // Multi-machine sync flag,0 for no sync
        temp[i][12] = 0x6B;             // End byte
    }
}

void Send_speed_switch(void)
{
    uart3WriteBuf(LB_send,8);
    Delay_ms(4);
    uart3WriteBuf(LF_send,8);
    Delay_ms(4);
    uart3WriteBuf(RB_send,8);
    Delay_ms(4);
    uart3WriteBuf(RF_send,8);
    Delay_ms(4);
}

// 左手坐标系运动解算，逆时针为正
void Motor_Action_Calculate_target(float vx, float vy, float vw) {
    __disable_irq();
    motor1.target_angle = vw + vy + vx; // 1号电机
    motor2.target_angle = vw + vy - vx; // 2号电机
    motor3.target_angle = vw - vy - vx; // 3号电机
    motor4.target_angle = vw - vy + vx; // 4号电机
    __enable_irq();
}

//延时分别发送速度指令
void Motor_setspeed(float vx, float vy, float vw)
{
    Motor_Action_Calculate_target(vx, vy, vw);
    Motor_Send_Speed_together(motor1.target_angle, motor2.target_angle, motor3.target_angle, motor4.target_angle);
    Send_speed_switch();
}

static uint8_t rx_buff1[RXdat_maxsize];
static uint8_t rx_buff2[RXdat_maxsize];

void USART3_Process_data(uint8_t* data, uint8_t len) {
    if (data[len-1] != 0x6B) {
        return;
    }
    if (len == 8) {
        switch (data[0])
        {
            case 0x01://
            {
                motor1.actual_angle = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6] ;
                motor1.actual_angle = motor1.actual_angle*360/65535;
                if (data[2] == 0x01) {
                    motor1.actual_angle = -motor1.actual_angle;
                }
                motor_check.flag_finish = motor_check.flag_finish | (1<<0);
                break;
            }
            case 0x02:
            {
                motor2.actual_angle = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6] ;
                motor2.actual_angle = motor2.actual_angle*360/65535;
                if (data[2] == 0x01) {
                    motor2.actual_angle = -motor2.actual_angle;
                }
                motor_check.flag_finish = motor_check.flag_finish | (1<<1);
                break;
            }
            case 0x03:
            {
                motor3.actual_angle = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6] ;
                motor3.actual_angle = motor3.actual_angle*360/65535;
                if (data[2] == 0x01) {
                    motor3.actual_angle = -motor3.actual_angle;
                }
                motor_check.flag_finish = motor_check.flag_finish | (1<<2);
                break;
            }
            case 0x04:
            {
                motor4.actual_angle = (data[3] << 24) | (data[4] << 16) | (data[5] << 8) | data[6] ;
                motor4.actual_angle = motor4.actual_angle*360/65535;
                if (data[2] == 0x01) {
                    motor4.actual_angle = -motor1.actual_angle;
                }
                motor_check.flag_finish = motor_check.flag_finish | (1<<3);
                break;
            }
        }
    }
    //速度控制模式就绪标志
    else if (len == 4)
    {
        if (data[0] == 0x01 && data[3] == 0x6B) {
            if (data[2] == 0xF6 && data[3] == 0x02)
            {
                motor_check.flag_ready = finish;
            }
        }
    }
}

// 接收完成标志检查，放在自定义中断处理函数中
void Motor_FinishFlag_Exam(uint8_t *RX_data) {
    uint8_t buff_len = 0;
    uint8_t* rxbuff = NULL;
    uint8_t rxbuff_flag = 0;

    // stop DMA to process received data
    if (HAL_UART_DMAStop(&huart3) != HAL_OK) {
        // error handling
        return;
    }

    // disable IRQ to ensure data consistency
    __disable_irq();

    // correctly get DMA reception counter
    if (huart3.hdmarx != NULL) {
        buff_len = RXdat_maxsize - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    }

    // copy received data to buffer
    if (buff_len > 0 && RX_data != NULL) {
        //双缓冲区切换
        if (rxbuff_flag == 0) {
            rxbuff = rx_buff1;
            rxbuff_flag = ~rxbuff_flag;
        }
        else {
            rxbuff = rx_buff2;
            rxbuff_flag = ~rxbuff_flag;
        }
        //复制数据到缓冲区
        memcpy(rxbuff, RX_data, buff_len);
        __enable_irq();
        USART3_Process_data(rxbuff,buff_len);
    }
    else {
        // 恢复中断（如果没有数据或指针无效）
        __enable_irq();
    }
}

void My_UART3_IRQHandler(void)
{
    Motor_FinishFlag_Exam(RX_data);
    //处理完数据重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RX_data, RXdat_maxsize);
}














