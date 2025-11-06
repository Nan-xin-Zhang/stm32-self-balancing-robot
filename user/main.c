#include "stm32f10x.h"
#include "app_control.h"
#include "app_motor.h"
#include "app_mpu6050.h"
#include "app_button.h"
#include "app_usart2.h"
#include "app_cmd.h"
#include "app_bat.h"
#include "app_lights.h"
#include "app_encoder_test.h"
#include "app_mpu6050_test.h"
#include "app_motor_test.h"
#include "app_pwm_test.h"
#include "app_bat_test.h"
#include "app_cmd_test.h"
#include "app_calibrator.h"


int main(void)
{
//	App_Encoder_DriftTest();
//	App_Cmd_Test();
//	App_Bat_Test();
//	App_PWM_Test();
//	MotorSpeedTest();
//  App_MPU6050_Test();
// 	App_Encoder_Test();
	App_Calibrator_Init();
	App_Bat_Init();
	App_USART2_Init();
	App_Button_Init();
	App_Motor_Init();
	App_MPU6050_Init();
	App_Control_Init();
	App_Cmd_Init();
	App_Lights_Init();
	
	while(1)
	{
		App_MPU6050_Proc();
		App_Bat_Proc();
		App_Motor_Proc();
		App_Control_Proc();
		App_Cmd_Proc();
		App_Lights_Proc();
		App_Button_Proc();
	}
}
