#include "app_cmd.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include "app_control.h"

void App_Cmd_Init(void)
{
	// #1. 初始化IO引脚
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	// PB10 AF_PP
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// PA3 IPU
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// #2. 开启USART3的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	// #3. 配置USART3的参数
	USART_InitTypeDef USART_InitStruct = {0};
	
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStruct);
	
	// #4. 配置RxNE中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
	
	// #5. 闭合USART3的总开关
	USART_Cmd(USART3, ENABLE);
}

static char buffer[64];
static char cmd[64];
static uint16_t cursor = 0;
static volatile uint8_t cmdPending = 0;

void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
	{
		uint8_t dataRcvd = USART_ReceiveData(USART3);
		
		if(dataRcvd == '\n')
		{
			buffer[cursor++] = '\0';
			strcpy(cmd, buffer);
			cursor = 0; // 清空
			cmdPending = 1;
		}
		else
		{
			if(cursor >= 254)
			{
				cursor = 0; // 清空
			}
			
			buffer[cursor++] = dataRcvd;
		}
	}
}

static void Move_Handler(const char *Args);

void App_Cmd_Proc(void)
{
	char cmdCpy[64];
	
	if(!cmdPending) return;
	
	__disable_irq();
	
	strcpy(cmdCpy, cmd); // 取出消息
	cmdPending = 0;
	
	__enable_irq();

	
	// 对消息进行解析
	
	// 将所有的空格替换成'\0'
	uint16_t len = strlen(cmdCpy);
	
	for(uint16_t i=0;i<len; i++)
	{
		if(cmdCpy[i] == ' ')
		{
			cmdCpy[i] = '\0';
		}
	}
	
	const char *name = cmdCpy;
	
	if(strcasecmp(name, "move") == 0)
	{
		Move_Handler(cmdCpy);
	}
}

static void Move(int8_t Speed, int8_t Turn)
{
//	My_USART_Printf(USART2, "%d,%d\n", Speed, Turn);
	
	App_Control_Move(Speed, Turn);
}

static void Move_Handler(const char *Args)
{
	const char *ptr = Args + strlen(Args) + 1;
	
	int8_t turn = atoi(ptr);
	
	ptr = ptr + strlen(ptr) + 1;
	
	int8_t speed = atoi(ptr);
	
	Move(speed, turn);
}
