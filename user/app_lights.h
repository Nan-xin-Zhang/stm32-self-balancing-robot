#ifndef APP_LIGHTS_H
#define APP_LIGHTS_H

#include "stm32f10x.h"

void App_Lights_Init(void);
void App_Lights_Cmd(uint8_t on);
void App_Lights_Proc(void);

#endif
