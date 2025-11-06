#include "app_bat.h"
#include "stm32f10x.h"
#include "task.h"
#include "usart.h"

static volatile uint8_t first_compute = 1;
static volatile float volt = 0;

static void TIM3_TRGO_Init(void);
static void ADC1_Init(void);
static void LEDs_Init(void);

void App_Bat_Init(void)
{
	ADC1_Init();
	TIM3_TRGO_Init();
	LEDs_Init();
}

static void LEDs_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void TIM3_TRGO_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {0};
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 9999; // 10ms
	TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	
	TIM_Cmd(TIM3, ENABLE);
}

void ADC1_2_IRQHandler(void)
{
	if(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == SET)
	{
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		
		uint16_t dr = ADC_GetConversionValue(ADC1);
		
		volt = dr / 4095.0f * 8.4;
	}
}

static void ADC1_Init(void)
{
	// #1. 初始化PB0 AIN
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// #2. 配置ADC1的时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	// #3. 配置ADC的基本参数
	ADC_InitTypeDef ADC_InitStruct = {0};
	
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel = 1;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	// #4. 配置常规序列
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	
	// #5. 使能EOC中断
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	
	NVIC_InitStruct.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
	
	// #6. 使能ADC
	ADC_Cmd(ADC1, ENABLE);
}

float App_Bat_Get(void)
{
	return volt;
}

void App_Bat_Proc(void)
{
	PERIODIC(20)
	
	// 5.8 6.6 7.4 8.2
	
	static uint32_t nxtBlinkTime = 0;
	static uint8_t blinkStage = 0; // 灭
	
	if(volt < 6) // 亏电，闪灯100ms
	{
		if(GetTick() > nxtBlinkTime)
		{
			nxtBlinkTime = GetTick() + 100;
			
			if(blinkStage == 0)
			{
				blinkStage = 1;
				// 全亮
				GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
				GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
				GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
			}
			else
			{
				blinkStage = 0;
				
				// 全灭
				GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
				GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
				GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
			}
		}
	}
	else if(volt < 6.6) // 亮0颗灯 <33 %
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
	}
	else if(volt < 7.3) // 亮1颗灯 <66 %
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
	}
	else if(volt < 8.0) // 亮2颗灯 <99 %
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET);
	}
	else // 全亮
	{
		GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
		GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET);
	}
}
