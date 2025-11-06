#include "app_control.h"
#include "app_pwm.h"
#include "app_encoder.h"
#include "app_bat.h"
#include "app_mpu6050.h"
#include "pid.h"
#include "task.h"
#include "qmath.h"
#include "usart.h"
#include "app_motor.h"

static PID_TypeDef pid_alpha;
static PID_TypeDef pid_dalpha;
static PID_TypeDef pid_vel;
static PID_TypeDef pid_turn;
static float omega_ref = 0;

static uint8_t standingUp = 0;

const float g = 9.8;   // 重力加速度

// 车体参数
static float rw = 0.032f; // 轮胎半径，单位m
static float lp = 0.062f; // 摆的长度，单位m
static float mp = 0.12f; // 摆的质量，单位kg
//static float mw = 0.26f; // 轮的质量，单位kg
static float Jp = 4.6128e-4f; // 摆的转动惯量

static void StartUp(void);

//static float rad_2_deg(float rad)
//{
//	return rad * 57.295779513f;
//}

static float deg_2_rad(float deg)
{
	return deg * 0.0174532925f;
}

void App_Control_Init(void)
{
	PID_InitTypeDef PID_InitStruct = {0};
	
	//
	// 速度环
	//
	PID_InitStruct.Kp = 0.2f;
	PID_InitStruct.Ki = 0.002f;
	PID_InitStruct.Kd = 0.0f;
	
	PID_InitStruct.DefaultOutput = 0;
	PID_InitStruct.Setpoint = 0;
	PID_InitStruct.OutputUpperLimit =  9.8f; // 输出量：加速度a，单位m/s^2，设其最大与重力加速度g相同
	PID_InitStruct.OutputLowerLimit = -9.8f;
	
	PID_Init(&pid_vel, &PID_InitStruct);
	
	//
	// 角度环
	//
	PID_InitStruct.Kp = 7;
	PID_InitStruct.Ki = 7;
	PID_InitStruct.Kd = 0;
	
	PID_InitStruct.DefaultOutput = 0;
	PID_InitStruct.Setpoint = 0;
	PID_InitStruct.OutputUpperLimit =  6.28f; // 输出量：角速度，单位rad/s，设其最大值为2PI rad/s
	PID_InitStruct.OutputLowerLimit = -6.28f;
	
	PID_Init(&pid_alpha, &PID_InitStruct);
	
	//
	// 角速度环
	//
	PID_InitStruct.Kp = 30;
	PID_InitStruct.Ki = 30;
	PID_InitStruct.Kd = 0;
	
	PID_InitStruct.DefaultOutput = 0;
	PID_InitStruct.Setpoint = 0;
	PID_InitStruct.OutputUpperLimit =  100.0f; // 输出量：角加速度，设其
	PID_InitStruct.OutputLowerLimit = -100.0f;
	
	PID_Init(&pid_dalpha, &PID_InitStruct);
	
	//
	// 转向环
	// 
	PID_InitStruct.Kp = 1;
	PID_InitStruct.Ki = 0;
	PID_InitStruct.Kd = 0;
	
	PID_InitStruct.DefaultOutput = 0;
	PID_InitStruct.Setpoint = 0;
	PID_InitStruct.OutputUpperLimit =  10.0f;
	PID_InitStruct.OutputLowerLimit = -10.0f;
	
	PID_Init(&pid_turn, &PID_InitStruct);
}

void App_Control_Proc(void)
{
	PERIODIC(5);
	
	if(standingUp) // 小车自动起立
	{
		StartUp();
		return; 
	}
	
	uint64_t now = GetUs();
	
	static uint64_t lastTime = 0;
	
	float deltaT = (now - lastTime) * 1.0e-6f;
	
	lastTime = now;
	
	// 采集传感器信息，角度和角速度
	
	float alpha = deg_2_rad(App_MPU6050_GetPitch()); // MPU6050传感器给出的是角度值，要转换成弧度值
	
	float dalpha = deg_2_rad(App_MPU6050_GetGyroX()); // rad/s
	
	float gz = deg_2_rad(App_MPU6050_GetGyroZ()); // rad/s
	
	///////////////////////////////////////////////////////////////////////
	// 速度环
	///////////////////////////////////////////////////////////////////////
	
	// 采集车轮速度
	float v = (App_Motor_GetSpeed_L() + App_Motor_GetSpeed_R()) / 2.0f + dalpha * (lp+rw) / rw;
	
	float alpha_ref = qatan(PID_Compute1(&pid_vel, v, now)/ g);
	
	
	///////////////////////////////////////////////////////////////////////
	// pid外环，角度环
	///////////////////////////////////////////////////////////////////////
	
	PID_ChangeSetpoint(&pid_alpha, alpha_ref);
	
	float dalpha_ref = PID_Compute1(&pid_alpha, alpha, now);
	
	///////////////////////////////////////////////////////////////////////
	// pid内环，角速度环
	///////////////////////////////////////////////////////////////////////
	
	PID_ChangeSetpoint(&pid_dalpha, dalpha_ref);
	
	float ddalpha_ref = PID_Compute1(&pid_dalpha, dalpha, now);
	
	// 解算，根据角加速度计算水平向加速度
	
	// float ddx_ref = (lp*ddalpha_ref - g*qsin(alpha)) / qcos(alpha);
	float ddx_ref = (Jp*ddalpha_ref - mp*g*lp*qsin(alpha)) / (mp*lp*qcos(alpha));
	
	// 对加速度积分，得到期望速度
	
	omega_ref += ddx_ref * deltaT / rw;
	
	if(omega_ref >  40) omega_ref = 40;
	if(omega_ref < -40) omega_ref = -40;
		
	///////////////////////////////////////////////////////////////////////
	// 转向环
	///////////////////////////////////////////////////////////////////////
	
	float omega_turn = PID_Compute1(&pid_turn, gz, now);
	
	if(fabsf(alpha) > deg_2_rad(80)) // 小车摔倒
	{
		standingUp = 5; // 进入自动起立模式
	}
	
	App_Motor_SetSpeed_L(-omega_ref + omega_turn);
	App_Motor_SetSpeed_R(-omega_ref - omega_turn);
}

void App_Control_Move(float speed, float turn)
{
	PID_ChangeSetpoint(&pid_vel, -speed / 3.8f);
	PID_ChangeSetpoint(&pid_turn, -turn / 7.0f);
}

static void StartUp(void)
{
	static uint32_t lastTime;
	
	switch(standingUp)
	{
		case 1: // 第1阶段，准备停转500ms
		{
			lastTime = GetTick();
			standingUp = 2; // 进入停转等待
			
			App_Motor_SetSpeed_L(0);
			App_Motor_SetSpeed_R(0);
			
			break;
		}
		case 2: // 停转等待
		{
			if(GetTick() - lastTime > 500)
			{
				standingUp = 3;
			}
			break;
		}
		case 3: // 最大功率反向转动
		{
			App_Motor_Reset();
				
			if(App_MPU6050_GetPitch() > 0) // 向前倒
			{
				App_Motor_SetSpeed_L(999);
				App_Motor_SetSpeed_R(999);
			}
			else // 向后倒
			{
				App_Motor_SetSpeed_L(-999);
				App_Motor_SetSpeed_R(-999);
			}
			
			lastTime = GetTick();
			
			standingUp = 4;
			
			break;
		}
		case 4: // 检测是否立起
		{
			if(fabsf(App_MPU6050_GetPitch()) < 40)
			{
				standingUp = 0;
				App_Control_Reset();
				App_Motor_Reset();
				break;
			}
			
			if(GetTick() - lastTime > 5000) // 超时，起立失败
			{
				App_Motor_Cmd(DISABLE);
				standingUp = 5;
			}
			break;
		}
		case 5: // 起立失败
			App_Motor_Cmd(DISABLE);
			break;
	}
}

void App_Control_Reset(void)
{
	PID_Reset(&pid_alpha);
	PID_Reset(&pid_dalpha);
	PID_Reset(&pid_vel);
	PID_Reset(&pid_turn);
	App_Motor_Reset();
	omega_ref = 0;
	standingUp = 0;
}
