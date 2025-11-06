#include "app_encoder.h"
#include "delay.h"
#include "math.h"
#include "app_calibrator.h"

static volatile int64_t encoder_l = 0; // 左电机编码器的值
static volatile int64_t encoder_r = 0; // 右电机编码器的值
static volatile int8_t d0_l = 0, d1_l = 0; // 左电机旋转的方向，0 - 初始状态，+-1234对应4个阶段
static volatile int8_t d0_r = 0, d1_r = 0; // 右电机旋转的方向，0 - 初始状态，+-1234对应4个阶段
static volatile uint64_t t0_l = 0, t1_l = 0; // 左电机编码器发生变化的时间，单位us
static volatile uint64_t t0_r = 0, t1_r = 0; // 右电机编码器发生变化的时间，单位us
static float m_l[2], m_r[2];
static void Encoder_L_Init(void); // 左编码器初始化
static void Encoder_R_Init(void); // 右编码器初始化

// 校准用
static volatile int8_t calibrateFlag_l = 0; // 校准开始标志位，0-校准停止，1-校准准备，2-校准开始，-1 - 校准失败
static volatile uint64_t lastEdge_l = 0; // 用于记录上次边沿发生的时间（校准用）
static volatile uint64_t t1_cali_l = 0, t2_cali_l = 0;
static volatile uint16_t n1_cali_l = 0, n2_cali_l = 0; 
// 校准用
static volatile int8_t calibrateFlag_r = 0; // 校准开始标志位，0-校准停止，1-校准准备，2-校准开始，-1 - 校准失败
static volatile uint64_t lastEdge_r = 0; // 用于记录上次边沿发生的时间（校准用）
static volatile uint64_t t1_cali_r = 0, t2_cali_r = 0;
static volatile uint16_t n1_cali_r = 0, n2_cali_r = 0; 

//
// @简介：对编码器模块进行初始化
//
void App_Encoder_Init(void)
{
	Encoder_L_Init(); 
	Encoder_R_Init(); 
	
	// 从校准器中读出占空比
	float duty_l = App_Calibrator_GetResult()->encoder_duty_l;
	float duty_r = App_Calibrator_GetResult()->encoder_duty_r;
	
	// 计算台阶高度
	// 左编码器台阶高度
	// m_l[0] - 第1阶段（从A相上升沿到A相下降沿）的台阶高度
	// m_1[1] - 第2阶段（从A相下降沿到A相上升沿）的台阶高度
	m_l[0] = duty_l * 2; // m[]表示台阶高度，占空比*2，比如占空比为0.49，则m[0]=0.98,m[1]=1.02
	m_l[1] = 2 - duty_l * 2;
	
	// 右编码器台阶高度
	m_r[0] = duty_r * 2;
	m_r[1] = 2 - duty_r * 2;
}

//
// @简介：读取左轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_L(void)
{
	// return encoder_l / 22.0f / (30613.0f / 1500.0f) * 2*PI; 
	return encoder_l * 0.01399402208920360588844895090594f;
}

//
// @简介：读取右轮胎旋转的角度，单位：度
//
float App_Encoder_GetPos_R(void)
{
	// return encoder_r / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
	return -encoder_r * 0.01399402208920360588844895090594f;
}

//
// @简介：读取左轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_L(void)
{
	__disable_irq(); // 为了保持数据完整性，关闭单片机的总中断
	
	// 拷贝传感器数据
	int8_t d0_cpy = d0_l; // 上一个时刻编码器方向及阶段
	int8_t d1_cpy = d1_l; // 上上个时刻编码器方向及阶段
	uint64_t t0_cpy = t0_l; // 上一个时刻
	uint64_t t1_cpy = t1_l; // 上上个时刻
	
	__enable_irq(); // 开启单片机的总中断
	
	if(d0_cpy * d1_cpy <= 0) return 0.0f; // 方向改变时强制令速度为0
	
	uint64_t now = GetUs(); // 获取当前时间
	
	int8_t dnow; // 当前方向
	
	// 推算当前阶段和方向
	if(d0_cpy > 0)
	{
		dnow = d0_cpy % 2 + 1; // 1->1%2+1=2 2->2%2+1=1
	}
	else
	{
		dnow = -((-d0_cpy) % 2 +1);
	}
	
	float M,T; // M表示台阶高度，T表示台阶宽度
	
	float Mnow = dnow > 0 ? m_l[dnow-1]   : -m_l[-dnow-1];
	float M0 = d0_cpy > 0 ? m_l[d0_cpy-1] : -m_l[-d0_cpy-1];
	
	if((fabsf(Mnow)*(t0_cpy-t1_cpy)) < fabsf(M0)*(now - t0_cpy)) // 速度减慢
	{	
		M = Mnow;
		T = (now - t0_cpy)  * 1.0e-6f;
	}
	else // 速度不变或者加快
	{
		M = M0;
		T = (t0_cpy-t1_cpy) * 1.0e-6f;
	}
	
	return M / T * 0.01399402208920360588844895090594f;
}


//
// @简介：读取右轮胎旋转的角速度，omega的值，单位是 度/s
// 
float App_Encoder_GetSpeed_R(void)
{
	__disable_irq(); // 关闭单片机的总中断
	
	int8_t d0_cpy = d0_r;
	int8_t d1_cpy = d1_r;
	uint64_t t0_cpy = t0_r;
	uint64_t t1_cpy = t1_r;
	
	__enable_irq(); // 开启单片机的总中断
	
	if(d0_cpy * d1_cpy <= 0) return 0.0f; // 方向改变时强制令速度为0
	
	uint64_t now = GetUs();
	int8_t dnow; // 当前方向
	
	if(d0_cpy > 0)
	{
		dnow = d0_cpy % 2 + 1; // 1->1%2+1=2 2->2%2+1=1
	}
	else
	{
		dnow = -((-d0_cpy) % 2 +1);
	}
	
	float M,T;
	
	float Mnow = dnow > 0 ? m_r[dnow-1]   : -m_r[-dnow-1];
	float M0 = d0_cpy > 0 ? m_r[d0_cpy-1] : -m_r[-d0_cpy-1];
	
	if((fabsf(Mnow)*(t0_cpy-t1_cpy)) < fabsf(M0)*(now - t0_cpy))
	{	
		M = Mnow;
		T = (now - t0_cpy)  * 1.0e-6f;
	}
	else
	{
		M = M0;
		T = (t0_cpy-t1_cpy) * 1.0e-6f;
	}
	
	return -M / T * 0.01399402208920360588844895090594f;
}

//
// @简介：左编码器初始化
//
static void Encoder_L_Init(void)
{
	// 初始化A和B的引脚
	// PB14, PB15 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);	
	
	// EXTI初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14); // 让EXTI_Line14监控PB14
	
	// 配置EXTI的参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	
	// EXTI14
	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	// 开启EXTI的中断EXTI15_10_IRQn
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;// 中断编号
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：右编码器初始化
//
static void Encoder_R_Init(void)
{
	// 初始化A和B的引脚
	// 关闭JTAG，开启SWD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	// PB3, PB4 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	// EXTI初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3); // 让EXTI_Line3监控PB3
	
	// 配置EXTI的参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	
	// EXTI3
	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	// 开启EXTI的中断
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	
	// EXTI3_IRQn
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;// 中断编号
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：EXTI3的中断响应函数，对应右编码器的A相
//
void EXTI3_IRQHandler(void)
{
	EXTI_ClearFlag(EXTI_Line3); // 对标志位进行清零
		
	uint8_t a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3); // A相的当前电压
	uint8_t b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4); // B相的当前电压
	
	uint64_t now = GetUs();
	
	t1_r = t0_r;
	t0_r = now;
	d1_r = d0_r;
	
	if(a == Bit_SET) // A相上升沿
	{
		if(b == Bit_SET) // 正转，1
		{
			d0_r = 2;
			encoder_r++;
			
			if(calibrateFlag_r == 1) // 校准准备中
			{
				calibrateFlag_r = 2; // 遇到1号边沿即开始校准
				
				// 对所有数据清零
				t1_cali_r = 0; t2_cali_r = 0;
				n1_cali_r = 0; n2_cali_r = 0;
				lastEdge_r = now;
			}
			else if(calibrateFlag_r == 2) // 遇到1号边沿，累加t4
			{
				t2_cali_r += now - lastEdge_r;
				n2_cali_r++;
				lastEdge_r = now;
			}
		}
		else // 反转
		{
			d0_r = -2;
			encoder_r--;
			
			if(calibrateFlag_r == 1 || calibrateFlag_r == 2)
			{
				calibrateFlag_r = -1; // 遇电机反转则校准失败
			}
		}
	}
	else // A相下降沿
	{
		if(b == Bit_RESET) // 正转，3
		{
			encoder_r++;
			d0_r = 1;
			
			if(calibrateFlag_r == 2) // 遇到1号边沿，累加t1
			{
				t1_cali_r += now - lastEdge_r;
				n1_cali_r++;
				lastEdge_r = now ;
			}
		}
		else // 反转
		{
			encoder_r--;
			d0_r = -1;
			
			if(calibrateFlag_r == 1 || calibrateFlag_r == 2)
			{
				calibrateFlag_r = -1; // 遇电机反转则校准失败
			}
		}
	}
}

//
// @简介：EXTI15_10的中断响应函数，对应左编码器的A相
//
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line14) == SET)
	{
		EXTI_ClearFlag(EXTI_Line14); // 对标志位进行清零
		
		uint8_t a = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14); // A相的当前电压
		uint8_t b = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15); // B相的当前电压
		
		uint64_t now = GetUs();
		
		t1_l = t0_l;
		t0_l = now;
		d1_l = d0_l;
		
		if(a == Bit_SET) // A相上升沿
		{
			if(b == Bit_SET) // 正转，1
			{
				d0_l = 2;
				encoder_l++;
				
				if(calibrateFlag_l == 1) // 校准准备中
				{
					calibrateFlag_l = 2; // 遇到1号边沿即开始校准
					
					// 对所有数据清零
					t1_cali_l = 0; t2_cali_l = 0;
					n1_cali_l = 0; n2_cali_l = 0;
					lastEdge_l = now;
				}
				else if(calibrateFlag_l == 2) // 累加t2
				{
					t2_cali_l += now - lastEdge_l;
					n2_cali_l++;
					lastEdge_l = now;
				}
			}
			else // 反转
			{
				d0_l = -2;
				encoder_l--;
				
				if(calibrateFlag_l == 1 || calibrateFlag_l == 2)
				{
					calibrateFlag_l = -1; // 遇电机反转则校准失败
				}
			}
		}
		else // A相下降沿
		{
			if(b == Bit_RESET) // 正转，3
			{
				encoder_l++;
				d0_l = 1;
				
				if(calibrateFlag_l == 2) // 累加t1
				{
					t1_cali_l += now - lastEdge_l;
					n1_cali_l++;
					lastEdge_l = now;
				}
			}
			else // 反转
			{
				encoder_l--;
				d0_l = -1;
				
				if(calibrateFlag_l == 1 || calibrateFlag_l == 2)
				{
					calibrateFlag_l = -1; // 遇电机反转则校准失败
				}
			}
		}
	}
}

void App_Encoder_StartCalibration(void)
{
	calibrateFlag_l = 1; // 准备校准
	calibrateFlag_r = 1; // 准备校准
	
	// 复位所有校准临时数据
	
	t1_cali_l = 0; t2_cali_l = 0;
	n1_cali_l = 0; n2_cali_l = 0;
	lastEdge_l = 0;
	
	t1_cali_r = 0; t2_cali_r = 0;
	n1_cali_r = 0; n2_cali_r = 0;
	lastEdge_r = 0;
}

int App_Encoder_EndCalibration(float *duty_l, float *duty_r)
{
	int ret = 0;
	
	__disable_irq();
	
	if(calibrateFlag_l < 0 || calibrateFlag_r < 0)
	{
		ret = -1;
	}
	
	calibrateFlag_l = 0; // 结束校准
	calibrateFlag_r = 0; // 结束校准
	
	__enable_irq();
	
	if(ret == 0)
	{
		float t1_l = ((float)t1_cali_l) / ((float)n1_cali_l);
		float t2_l = ((float)t2_cali_l) / ((float)n2_cali_l);
		
		float t1_r = ((float)t1_cali_r) / ((float)n1_cali_r);
		float t2_r = ((float)t2_cali_r) / ((float)n2_cali_r);
		
		*duty_l = t1_l / (t1_l + t2_l);
		*duty_r = t1_r / (t1_r + t2_r);
	}
	else
	{
		*duty_l = 0.5;
		*duty_r = 0.5;
	}
	
	return ret;
}
