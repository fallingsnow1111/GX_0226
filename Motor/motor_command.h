#ifndef __MOTOR_COMMAND_H
#define __MOTOR_COMMAND_H

#include "usart.h"
#include "dma.h"
#include "Struct_encapsulation.h"

#define RXdat_maxsize 128

extern volatile struct CHECK_FLAG_t motor_check;

// extern DMA_HandleTypeDef hdma_usart3_rx;

struct MOTOR_DATA {
    float target_angle;
    float actual_angle;
};

typedef struct {
    uint32_t rx_event_cnt;
    uint32_t rx_byte_cnt;
    uint32_t pos_frame_ok_cnt[4];
    uint32_t ack_frame_ok_cnt;
    uint32_t invalid_id_cnt;
    uint32_t invalid_cmd_cnt;
    uint32_t invalid_tail_cnt;
    uint32_t rx_cache_overflow_cnt;
} MOTOR_UART3_DEBUG_t;

extern volatile MOTOR_UART3_DEBUG_t motor_uart3_debug;

void Motor_Init(void);
void Motor_Send_Speed_together(float LB, float LF, float RF, float RB);
void Motor_setspeed(float vx, float vy, float vw);
void Motor_setposition_relative(float dx, float dy, float dw);
void Motor_setposition_absolute_zero(float x, float y, float w);
void Motor_SetZero(void);
void Send_motor_together(void);
void Motor_read_coordination(uint8_t motor_id);
void Send_Position_together(int LB, int LF, int RF, int RB, char mode);

void My_UART3_IRQHandler(uint16_t Size);


#endif
