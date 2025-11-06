#include "app_motor.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "pid.h"
#include "delay.h"
#include "task.h"
#include "math.h"
#include "app_bat.h"

// 电机参数
//static const float La = 1.5e-3f; // 电枢电感，单位H
//static const float Ra = 3.0f; // 电枢电阻，单位Ω
//static const float Kt = 0.176; // 扭矩常数，单位N.m/A
//static const float Ke = 0.176; // 反电动势常数，单位V/(rad/s)
//static const float Jm = 2.787e-4f; // 电机的转动惯量，单位kg.m^2
//static const float Jw = 3.891e-4f; // 小车等效转动惯量
//static const float B  = 2.279e-4f; // 电机摩擦系数，单位N.m/(rad/s)
//static const float TIdle = 0.01f; // 负载力矩，单位N.m

static PID_TypeDef pid_l; // 左电机速度环PID
static PID_TypeDef pid_r; // 右电机速度环PID

static FunctionalState enabled = DISABLE; // 电机的当前使能状态

void App_Motor_Init(void)
{
	App_PWM_Init();
	App_Encoder_Init();
	
	PID_InitTypeDef PID_InitStruct = {0};
	
	PID_InitStruct.Kp = 0.5f;
	PID_InitStruct.Ki = 5.0f;
	PID_InitStruct.Kd = 0.0f;
	PID_InitStruct.OutputUpperLimit = 8.2f; // 最高电压8.2V
	PID_InitStruct.OutputLowerLimit = -8.2f; // 最低电压-8.2V
	PID_InitStruct.Setpoint = 0;
	PID_InitStruct.DefaultOutput = 0;
	
	PID_Init(&pid_l, &PID_InitStruct);
	PID_Init(&pid_r, &PID_InitStruct);
}

void App_Motor_Cmd(FunctionalState NewState)
{
	App_PWM_Cmd(NewState);
	PID_Cmd(&pid_l, NewState);
	PID_Cmd(&pid_r, NewState);
	
	enabled = NewState;
}

FunctionalState App_Motor_GetState(void)
{
	return enabled;
}

void App_Motor_Reset(void)
{
	PID_Reset(&pid_l);
	PID_Reset(&pid_r);
}

void App_Motor_Proc(void)
{
	PERIODIC(1)
	
	uint64_t now = GetUs();
	
	// 编码器
	float omega_l = App_Encoder_GetSpeed_L(); // 左轮转速，单位rad/s
	float omega_r = App_Encoder_GetSpeed_R(); // 右轮转速，单位rad/s
	
	// 电池电压
	float bat = App_Bat_Get();
	
	// PID
	float Va_l = PID_Compute1(&pid_l, omega_l, now);
	float Va_r = PID_Compute1(&pid_r, omega_r, now);
	
	// 由期望电压计算占空比
	float duty_l = Va_l / bat * 100.0;
	float duty_r = Va_r / bat * 100.0;
	
	App_PWM_Set_L(duty_l);
	App_PWM_Set_R(duty_r);
}

float App_Motor_GetSpeed_L(void)
{
	return App_Encoder_GetSpeed_L();
}

float App_Motor_GetSpeed_R(void)
{
	return App_Encoder_GetSpeed_R();
}

//
// @简介：设置左电机的转速，单位rad/s
//        该电机的标称转速为282r/min（29.516rad/s max）
//
void App_Motor_SetSpeed_L(float Speed)
{
	PID_ChangeSetpoint(&pid_l, Speed);
}

//
// @简介：设置右电机的转速，单位rad/s
//        该电机的标称转速为282r/min（29.516rad/s max）
//
void App_Motor_SetSpeed_R(float Speed)
{
	PID_ChangeSetpoint(&pid_r, Speed);
}
