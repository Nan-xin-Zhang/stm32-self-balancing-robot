#include "app_calibrator.h"
#include "delay.h"
#include "app_encoder.h"
#include "app_pwm.h"
#include "task.h"
#include "app_mpu6050.h"

#define CALI_RESULT_ADDR_START 0x0801fC00 // 存储校准结果的起始位置，Page127
#define CALI_KEY 0x34562897feda0312

static CaliResult_TypeDef caliResult; // 用于存储校准信息

static void OnBoardLED_Init(void);
static void OnBoardLED_Set(uint8_t State);
static int CalibrateEncoders(void); // 编码器校准
static int CalibrateMPU6050(void); // 校准MPU6050
static void LoadCaliResult(void); // 从单片机的Flash加载校准信息
static void SaveCaliResult(void); // 将校准信息保存回单片机的Flash当中

//
// @简介：初始化校准器
//
void App_Calibrator_Init(void)
{
	OnBoardLED_Init(); // 初始化板载LED，校准时作为指示灯
	LoadCaliResult(); // 加载之前存储的校准结果
}

//
// @简介：执行校准过程
//
void App_Calibrator_DoCalibration(void)
{
	
	// #1. 等待用户将设备放置到合适位置
	// 板载LED闪烁，表示即将进入校准状态
	for(uint32_t i=0; i<50;i++) // 5s
	{
		OnBoardLED_Set(0);
		Delay(50);
		OnBoardLED_Set(1);
		Delay(50);
	}
	
	// #2. 点亮LED，开始校准过程
	OnBoardLED_Set(1);
	
	// #2.1. 校准编码器
	if(CalibrateEncoders() != 0)
	{
		// 校准失败
		goto CALI_END;
	}
	
	// #2.2. 校准MPU6050
	if(CalibrateMPU6050() != 0)
	{
		goto CALI_END;
	}
	
	// #3. 保存校准结果
	SaveCaliResult();
	
CALI_END:
	// #4. 校准结束，持续闪灯
	while(1)
	{
		OnBoardLED_Set(0);
		Delay(50);
		OnBoardLED_Set(1);
		Delay(50);
	}
}

const CaliResult_TypeDef *App_Calibrator_GetResult(void)
{
	return &caliResult;
}

//
// @简介：初始化板载LED，将板载LED作为校准指示灯
//
static void OnBoardLED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET); // 初始状态下熄灭板载LED
	
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

//
// @简介：控制板载LED的亮灭
// @参数：State - 0 - 熄灭，非零 - 点亮
//
static void OnBoardLED_Set(uint8_t State)
{
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, State ? Bit_RESET : Bit_SET);
}

//
// @简介：校准编码器误差
// @返回值：0 - 成功 非零 - 失败
//
static int CalibrateEncoders(void)
{
	int ret = 0;
	
	// #1. 开启电机
	App_PWM_Cmd(1);
	App_PWM_Set_L(50);
	App_PWM_Set_R(-50);
	
	// #2. 等待1秒
	Delay(1000);
	
	// #3. 开始测试
	App_Encoder_StartCalibration();
	
	// #4. 等待10秒
	Delay(10000);
	
	// #5. 校准结束
	float duty_l, duty_r;
	if(App_Encoder_EndCalibration(&duty_l, &duty_r) != 0)
	{
		ret = -1; // 校准失败
	}
	
	// #7. 关闭电机
	App_PWM_Cmd(0);
	App_PWM_Set_L(0);
	App_PWM_Set_R(0);
	
	// #8. 写入校准结果
	if(ret == 0) // 只有校准成功才写入
	{
		caliResult.encoder_duty_l = duty_l;
		caliResult.encoder_duty_r = duty_r;
	}
	
	return ret;
}

//
// @简介：校准MPU6050
//
static int CalibrateMPU6050(void)
{
	// #1. 等待1s
	Delay(1000);
	
	// #2. 将之前的校准数据清零
	caliResult.mpu6050_gx_bias = 0;
	caliResult.mpu6050_gy_bias = 0;
	caliResult.mpu6050_gz_bias = 0;
	caliResult.mpu6050_pitch_bias = 0;
	
	// #3. 进入校准，假定采样率200Hz，10s对应2000个点
	
	uint16_t n;
	float gx = 0, gy = 0, gz = 0, pitch = 0;
	
	while(1)
	{
		App_MPU6050_Proc();
		
		PERIODIC_START(MPU6050_AQUIRE, 5) // 每5ms采集一次数据
		
		gx+= App_MPU6050_GetGyroX();
		gy+= App_MPU6050_GetGyroY();
		gz+= App_MPU6050_GetGyroZ();
		
		float tmp = App_MPU6050_GetPitch() - 180;
		
		if(tmp < -180) // 将角度限制在-180到180之间
		{
			tmp += 360;
		}
		else if(tmp > 180)
		{
			tmp -= 360;
		}
		
		pitch += tmp;
		n++;
		
		if(n==2000) break;
		
		PERIODIC_END
	}
	
	caliResult.mpu6050_gx_bias = gx / n;
	caliResult.mpu6050_gy_bias = gy / n;
	caliResult.mpu6050_gz_bias = gz / n;
	caliResult.mpu6050_pitch_bias = pitch / n;
	
	return 0;
}

static void LoadCaliResult(void)
{
	// #1. 加载校准结构
	// #1.1. 从单片机的Flash加载校准结果
	caliResult = *(CaliResult_TypeDef *)CALI_RESULT_ADDR_START;
	
	// #1.2. 校验key，以确定校准信息是否有效
	if(caliResult.key != CALI_KEY) // 如果校准信息无效
	{
		// 给校准信息赋默认值
		caliResult.encoder_duty_l = 0.5f;
		caliResult.encoder_duty_r = 0.5f;
		caliResult.mpu6050_gx_bias = 0;
		caliResult.mpu6050_gy_bias = 0;
		caliResult.mpu6050_gz_bias = 0;
		caliResult.mpu6050_pitch_bias = 0;
	}
}

//
// @简介：将校准结果保存到单片机的Flash当中
//
static void SaveCaliResult(void)
{
	// #1. 解锁Flash
	FLASH_Unlock();
	
	// #2. 页擦除
	FLASH_ErasePage(CALI_RESULT_ADDR_START); // 擦除page127
	
	// #3. 页编程
	caliResult.key = CALI_KEY;
	uint16_t *data = (uint16_t *)&caliResult;
	uint16_t size = sizeof(caliResult)/2 + 1;
	
	for(uint16_t i=0; i<size; i++)
	{
		FLASH_ProgramHalfWord(CALI_RESULT_ADDR_START+2*i, data[i]);
	}
	
	// #4. 锁定Flash
	FLASH_Lock();
}
