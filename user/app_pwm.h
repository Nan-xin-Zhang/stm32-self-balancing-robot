#ifndef APP_MOTOR_PWM_H
#define APP_MOTOR_PWM_H

#include "stm32f10x.h"

void App_PWM_Init(void);
void App_PWM_Cmd(uint8_t State);
void App_PWM_Set_L(float Duty);
void App_PWM_Set_R(float Duty);

#endif
