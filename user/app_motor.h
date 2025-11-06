#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f10x.h"

void App_Motor_Init(void);
void App_Motor_Cmd(FunctionalState NewState);
FunctionalState App_Motor_GetState(void);
void App_Motor_Reset(void);
void App_Motor_Proc(void);
void App_Motor_SetSpeed_L(float speed);
void App_Motor_SetSpeed_R(float speed);
float App_Motor_GetSpeed_L(void);
float App_Motor_GetSpeed_R(void);

#endif
