#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "cmsis_os.h"
#include "Struct_encapsulation.h"

extern volatile uint8_t MOTOR_ACTION_FINISH_FLAG;
extern volatile CARDATA_T car;

void chassis_control_init(void);
void chassis_control(void);
void Get_Chassis_c_output(float *x_output, float *y_output, float *w_output);
void Odometer_SetZero(void);
void Chassis_SetRelativeZero(void);
void Chassis_SetRelativeTarget(float dx, float dy, float dw);

#endif
