#include "motor_control.h"

float my_Abs(float temp)
{
    if (temp < 0) return -temp;
    else return temp;
}

void motor_read_coordination_all(void)
{
    for (int i = 1; i <= 4; i++)
    {
        Motor_read_coordination(i);
        Delay_ms(1);
    }
}
