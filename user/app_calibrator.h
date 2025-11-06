#ifndef APP_CALIBRATOR_H
#define APP_CALIBRATOR_H

#include "stm32f10x.h"

/*
* @简介：该结构体用于存储校准参数
*/
typedef struct
{
	uint64_t key; // 用于校验是否存在有效的校准信息，当该值等于CALIBRATOR_KEY时表示校准信息有效
	
	// 编码器校准相关
	float encoder_duty_l; // 左编码器占空比
	float encoder_duty_r; // 右编码器占空比
	float mpu6050_gx_bias; // 陀螺仪x轴偏移量
	float mpu6050_gy_bias;
	float mpu6050_gz_bias;
	float mpu6050_pitch_bias;
	
} CaliResult_TypeDef;

void App_Calibrator_Init(void);
void App_Calibrator_DoCalibration(void);
const CaliResult_TypeDef *App_Calibrator_GetResult(void);

#endif
