#include "app_pwm.h"
#include "qmath.h"

#define PERIOD 999

static void StbyPin_Init(void);
static void PWM_L_Init(void);
static void PWM_R_Init(void);

void App_PWM_Init(void)
{
	StbyPin_Init();
	PWM_L_Init();
	PWM_R_Init();
}

void App_PWM_Cmd(uint8_t State)
{
	if(State)
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
	}
	else
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
	}
}

void App_PWM_Set_L(float Duty)
{
	if(Duty >  100) Duty = 100;
	if(Duty < -100) Duty = -100;
	
	if(Duty >= 0)
	{
		// IN1 高，IN2 低
		GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_RESET);
	}
	else
	{
		// IN1 低，IN2 高
		GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_9, Bit_SET);
	}
	
	TIM_SetCompare1(TIM1, (uint16_t)(fabsf(Duty) / 100.0f * (PERIOD+1)));
}

void App_PWM_Set_R(float Duty)
{
	if(Duty >  100) Duty = 100;
	if(Duty < -100) Duty = -100;
	
	if(Duty >= 0)
	{
		// IN1 高，IN2 低
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
	}
	else
	{
		// IN1 低，IN2 高
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
	}
	
	TIM_SetCompare1(TIM4, (uint16_t)(fabsf(Duty) / 100.0f * (PERIOD + 1)));
}

static void PWM_L_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// #1. 初始化IO引脚 
	// PA9  Out_PP 
	// PA10 Out_PP
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// #2. 使能定时器1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	// #3. 配置定时器的基本参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0; // 72/18=4
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
	
	// #4. 配置输出比较
	TIM_OCInitTypeDef TIM_OCInitStruct = {0};
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = ENABLE;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// #5. 开启CCR的预加载
	TIM_CCPreloadControl(TIM1, ENABLE);
	
	// #6. 闭合总开关
	TIM_Cmd(TIM1, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

static void PWM_R_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// #1. 初始化IO引脚 
	// PB5 Out_PP 
	// PB7 Out_PP
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// #2. 使能定时器4的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	// #3. 配置定时器的基本参数
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = PERIOD;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	// #4. 配置输出比较
	TIM_OCInitTypeDef TIM_OCInitStruct = {0};
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState = ENABLE;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OC1Init(TIM4, &TIM_OCInitStruct);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// #5. 开启CCR的预加载
	TIM_CCPreloadControl(TIM4, ENABLE);
	
	// #6. 闭合总开关
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
}

static void StbyPin_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}
