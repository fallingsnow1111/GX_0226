#include "IMU.h"

#include "imu_control.h"

#define RING_BUFFER_SIZE 128
#define IMU_FRAME_SIZE 11

//配置命令数组
//解锁寄存器 FF AA 69 88 B5
//Z轴角度归零FF AA 76 00 00
//设置200HZ输出 FF AA 03 0B 00
//设置波特率115200 FF AA 04 06 00
//保存 FF AA 00 00 00
//重启 FF AA 00 FF 00
uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};

struct IMU imu;
static uint8_t imu_buffer[RING_BUFFER_SIZE];
static uint16_t read_index = 0;
uint8_t mpu_flash = 0;  //暂时不知道作用

void  U2_send(uint8_t data)
{
    uint16_t Usart2_time = 0;
    USART2->TDR = data;
    while((USART2->ISR & USART_ISR_TXE_Msk) == 0)    //USART_ISR_TXE=1表示发送寄存器为空
    {
        Usart2_time++;
        if (Usart2_time > 65535)
        {
            break;
        }
    }
    //清除串口接收端溢出错误标志位，防止外设卡死
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

void U2_writebuf(uint8_t* buf, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        U2_send(buf[i]);
    }
}

static uint16_t RingBuffer_GetCount(uint16_t write_idx)
{
    if (write_idx >= read_index)
    {
        return write_idx - read_index;
    }

    return RING_BUFFER_SIZE - read_index + write_idx;
}

static uint8_t RingBuffer_Peek(uint16_t offset)
{
    return imu_buffer[(read_index + offset) % RING_BUFFER_SIZE];
}

void IMU_Receive_Init(void)
{
    read_index = 0;
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    if (HAL_UART_Receive_DMA(&huart2, imu_buffer, RING_BUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}

void IMU_SetZero(void)
{
    U2_writebuf(reset_z_axis,5);
    imu_run.IS_MOVING = 0;
    imu_run.LAST_ANGLE = 0;
}

void Imu_unlock_register(void)
{
    U2_writebuf(unlock_register,5);
}

void Imu_setset_baudrate_115200(void)
{
    U2_writebuf(set_baudrate_115200,5);
}

void Imu_setsave_settings(void)
{
    U2_writebuf(save_settings,5);
}
void Imu_set200hz(void)
{
    U2_writebuf(set_output_200Hz,5);
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        uint16_t write_idx = RING_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

        while (RingBuffer_GetCount(write_idx) >= IMU_FRAME_SIZE)
        {
            if (RingBuffer_Peek(0) == 0x55 && RingBuffer_Peek(1) == 0x53)
            {
                uint8_t sum = 0;
                for (int i = 0; i < 10; i++)
                {
                    sum += RingBuffer_Peek(i);
                }

                if (sum == RingBuffer_Peek(10))
                {
                    uint8_t low = RingBuffer_Peek(6);
                    uint8_t high = RingBuffer_Peek(7);
                    int16_t raw_yaw = (int16_t)(((uint16_t)high << 8) | low);
                    float new_yaw = (float)raw_yaw / 32768.0f * 180.0f;

                    imu.yaw = 0.9f * new_yaw + 0.1f * imu.yaw;
                    mpu_flash = ~mpu_flash;

                    read_index = (read_index + IMU_FRAME_SIZE) % RING_BUFFER_SIZE;
                    continue;
                }
            }

            read_index = (read_index + 1) % RING_BUFFER_SIZE;
        }
    }
}














