#include "app_lights.h"
#include "stm32f10x.h"
#include "delay.h"
#include "task.h"

typedef struct{
	uint8_t R[8];
	uint8_t G[8];
	uint8_t B[8];
}RGB;

typedef struct
{
	uint8_t dummy1;
	
	RGB rgbs[5];
	
	uint8_t dummy2;
}RGBs;

#pragma pack()

static RGBs rgbs;

static void Downmap(void);
static void Set(uint8_t Which, uint8_t R, uint8_t G, uint8_t B);

void App_Lights_Init(void)
{
	// #1. 使能TIM2的时钟
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	// #2. 初始化TIM2的时基单元
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_Period = 63;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	// #3. 配置DMA，使用UEV触发DMA传输
	TIM_DMAConfig(TIM2, TIM_DMABase_CCR1, TIM_DMABurstLength_1Transfer);
	TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
	
	// #4. 配置输出比较
	
	// 初始化IO引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; // PWM模式1
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
	
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);
	
	TIM_Cmd(TIM2, ENABLE);
}

static uint8_t enabled = 1;

void App_Lights_Cmd(uint8_t on)
{
	enabled = on;
}

static uint8_t stage = 0;

void App_Lights_Proc(void)
{
	PERIODIC(100)
	
	for(uint32_t i=0;i<5;i++)
	{
		Set(i, 0, 0, 0);
	}
	
	if(enabled)
	{
		Set(stage, 255, 255, 255);
		
		stage++;
		
		if(stage >= 5) stage = 0;
	}
	
	Downmap();
}

#define CODE_ZERO_H_TIME 21
#define CODE_ONE_H_TIME  43

static void Set(uint8_t Which, uint8_t R, uint8_t G, uint8_t B)
{
	uint32_t i;
	
	for(i=0;i<8;i++)
	{
		rgbs.rgbs[Which].R[i] = (R & (1<<(7-i))) ? CODE_ONE_H_TIME : CODE_ZERO_H_TIME;
	}
	
	for(i=0;i<8;i++)
	{
		rgbs.rgbs[Which].G[i] = (G & (1<<(7-i))) ? CODE_ONE_H_TIME : CODE_ZERO_H_TIME;
	}
	
	for(i=0;i<8;i++)
	{
		rgbs.rgbs[Which].B[i] = (B & (1<<(7-i))) ? CODE_ONE_H_TIME : CODE_ZERO_H_TIME;
	}
}

static void Downmap(void)
{
//	DelayUs(80); // Reset
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_Cmd(DMA1_Channel2, DISABLE);
	
	DMA_InitTypeDef DMA_InitStruct;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&TIM2->DMAR;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&rgbs;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStruct.DMA_BufferSize = sizeof(rgbs);
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStruct); // DMA1_CH2
	
	DMA_ClearFlag(DMA1_FLAG_GL2);
	
	DMA_Cmd(DMA1_Channel2, ENABLE);
	
//	while(DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET);
	
//	DMA_Cmd(DMA1_Channel2, DISABLE);
	
//	DelayUs(80); // Reset
}
