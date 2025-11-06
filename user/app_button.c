#include "app_button.h"
#include "button.h"
#include "app_motor.h"
#include "app_control.h"
#include "app_lights.h"
#include "app_calibrator.h"

Button_TypeDef UserButton;

static void UserButtonClickCb(uint8_t Clicks);
static void LongPressCb(uint8_t ticks);

void App_Button_Init(void)
{
	Button_InitTypeDef Button_InitStruct = {0};
	
	Button_InitStruct.GPIOx = GPIOA;
	Button_InitStruct.GPIO_Pin = GPIO_Pin_11;
	
	My_Button_Init(&UserButton, &Button_InitStruct);
	
	My_Button_SetClickCb(&UserButton, UserButtonClickCb);
	My_Button_SetLongPressCb(&UserButton, LongPressCb);
}

void App_Button_Proc(void)
{
	My_Button_Proc(&UserButton);
}

static uint8_t lightsOn = 0;

static void UserButtonClickCb(uint8_t Clicks)
{
	if(Clicks == 1)
	{
		FunctionalState motorState;
		
		motorState = App_Motor_GetState();
		
		App_Control_Reset();
		App_Motor_Reset();
		
		if(motorState == DISABLE)
		{
			App_Motor_Cmd(ENABLE);
		}
		else
		{
			App_Motor_Cmd(DISABLE);
		}
	}
	else if(Clicks == 2) // 连点2下开关彩灯
	{
		lightsOn = !lightsOn;
		
		App_Lights_Cmd(lightsOn);
	}
}

static void LongPressCb(uint8_t ticks)
{
	if(ticks == 1)
	{
		App_Calibrator_DoCalibration(); // 长按进入校准
	}
}
