#include "delay.h"

uint16_t delay_time = 0;

void Delay_Init(void)
{
    HAL_TIM_Base_Start(&htim6);
    __HAL_TIM_ENABLE_IT(&htim6,TIM_IT_UPDATE);
}

//比HAL_Delay更高精度的延时;S
void Delay_ms(uint16_t ms)
{
    delay_time = 0;
    if (delay_time <= ms) {};
}


