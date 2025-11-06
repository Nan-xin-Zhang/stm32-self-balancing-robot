#include "app_mpu6050.h"
#include "si2c.h"
#include "task.h"
#include "qmath.h"
#include "app_calibrator.h"

static SI2C_TypeDef si2c;

static void    reg_write(uint8_t reg, uint8_t data);
static uint8_t reg_read(uint8_t reg);

void App_MPU6050_Init(void)
{
	// #1. 初始化软I2C PB8-SCL  PB9-SDA
	si2c.SCL_GPIOx = GPIOB;
	si2c.SCL_GPIO_Pin = GPIO_Pin_8;
	si2c.SDA_GPIOx = GPIOB;
	si2c.SDA_GPIO_Pin = GPIO_Pin_9;
	
	My_SI2C_Init(&si2c);
	
	// #2. 初始化MPU6050
	reg_write(0x6b, 0x80); // 设备复位
	Delay(100);
	reg_write(0x6b, 0x01); // 关闭睡眠模式，并将陀螺仪作为时钟来源
	reg_write(0x19, 0x00); // 设置采样率为1kHz
	
	reg_write(0x1b, 0x18); // 设置陀螺仪的量程为2000°/s
	reg_write(0x1a, 0x02); // 设置陀螺仪的带宽为94Hz
	// reg_write(0x1a, 0x00); // 设置陀螺仪的带宽为250Hz
	
	reg_write(0x1c, 0x00); // 设置加速度传感器的量程为2g
	reg_write(0x1d, 0x02); // 设置加速度传感器的带宽为92Hz
	// reg_write(0x1d, 0x00); // 设置加速度传感器的带宽为460Hz
}

static uint8_t firstCompute = 1;
static float ax, ay, az, temp, gx, gy, gz, yaw, roll, pitch;

void App_MPU6050_Proc(void)
{
	PERIODIC(5); // 每5ms执行一次
	App_MPU6050_Update();
}

void App_MPU6050_Update(void)
{
	// #1. 读取传感器原始值
	int16_t accel_x_raw = (short)(reg_read(0x3b) << 8) | reg_read(0x3c);
	int16_t accel_y_raw = (short)(reg_read(0x3d) << 8) | reg_read(0x3e);
	int16_t accel_z_raw = (short)(reg_read(0x3f) << 8) | reg_read(0x40);
	
	int16_t temp_raw = (short)(reg_read(0x41) << 8) | reg_read(0x42);
	
	int16_t gyro_x_raw = (short)(reg_read(0x43) << 8) | reg_read(0x44);
	int16_t gyro_y_raw = (short)(reg_read(0x45) << 8) | reg_read(0x46);
	int16_t gyro_z_raw = (short)(reg_read(0x47) << 8) | reg_read(0x48);
	
	// #2. 换算
	ax = accel_x_raw * 0.00006103515625f;
	ay = accel_y_raw * 0.00006103515625f;
	az = accel_z_raw * 0.00006103515625f;
	
	temp = temp_raw * 0.00294117647059f + 36.53;
	
	gx = gyro_x_raw * 0.06097560975610f - App_Calibrator_GetResult()->mpu6050_gx_bias;
	gy = gyro_y_raw * 0.06097560975610f - App_Calibrator_GetResult()->mpu6050_gy_bias;
	gz = gyro_z_raw * 0.06097560975610f - App_Calibrator_GetResult()->mpu6050_gz_bias;
	
	// #3. 互补滤波器
	
	float pitch_accel = qatan2(ay, az) * 1.0f / 3.14159265f * 180.0f;    // 翻滚角（加速度）
	float roll_accel = -qatan2(ax, az) * 1.0f / 3.14159265f * 180.0f;  // 俯仰角（加速度）
	
	if(firstCompute) // 首次融合时使用加速度结果
	{
		firstCompute = 0;
		
		yaw = 0;
		roll = roll_accel;
		pitch = pitch_accel;
	}
	else
	{
		// 计算偏航角
		yaw = yaw + gz * 0.005; // 直接使用陀螺仪计算偏航角
		
		// 计算翻滚角
		if(roll - roll_accel > 180) roll -= 360;
		if(roll_accel - roll > 180) roll += 360;
		roll = 0.95238 * (roll + gy * 0.005) + (1 - 0.95238) * roll_accel;
		
		// 计算俯仰角
		if(pitch - pitch_accel > 180) pitch -= 360;
		if(pitch_accel - pitch > 180) pitch += 360;
		pitch = 0.95238 * (pitch + gx * 0.005) + (1 - 0.95238) * pitch_accel;
	}
	
	// 将所有角度限制在+-180度之间
	if(yaw > 180) yaw -= 360;
	if(yaw < -180) yaw += 360;
	
	if(roll > 180) roll -= 360;
	if(roll < -180) roll += 360;
	
	if(pitch > 180) pitch -= 360;
	if(pitch < -180) pitch += 360;
}

float App_MPU6050_GetAccelX(void)
{
	return ax; 
}

// 单位 度/s
float App_MPU6050_GetAccelY(void)
{ 
	return ay; 
}

float App_MPU6050_GetAccelZ(void)
{ 
	return az; 
}

float App_MPU6050_GetGyroX(void)
{
	return gx; 
}

float App_MPU6050_GetGyroY(void)
{ 
	return gy; 
}

float App_MPU6050_GetGyroZ(void)
{ 
	return gz; 
}

float App_MPU6050_GetTemperature(void)
{ 
	return temp; 
}

float App_MPU6050_GetYaw(void)
{
	return yaw; 
}

float App_MPU6050_GetRoll(void)
{ 
	return roll; 
}

float App_MPU6050_GetPitch(void)
{ 
	return pitch + App_Calibrator_GetResult()->mpu6050_pitch_bias; 
}

static void reg_write(uint8_t reg, uint8_t data)
{
	My_SI2C_RegWriteBytes(&si2c, 0xd0, reg, &data, 1);
}

static uint8_t reg_read(uint8_t reg)
{
	uint8_t regVal;
	
	My_SI2C_RegReadBytes(&si2c, 0xd0, reg, &regVal, 1);
	
	return regVal;
}
