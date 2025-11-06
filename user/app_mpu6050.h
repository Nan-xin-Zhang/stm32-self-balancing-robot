#ifndef MPU6050_H
#define MPU6050_H

#include "stdint.h"

 void App_MPU6050_Init(void);
 void App_MPU6050_Proc(void);
 void App_MPU6050_Update(void);
float App_MPU6050_GetAccelX(void);
float App_MPU6050_GetAccelY(void);
float App_MPU6050_GetAccelZ(void);
float App_MPU6050_GetGyroX(void);
float App_MPU6050_GetGyroY(void);
float App_MPU6050_GetGyroZ(void);
float App_MPU6050_GetTemperature(void);
float App_MPU6050_GetYaw(void);
float App_MPU6050_GetRoll(void);
float App_MPU6050_GetPitch(void);

#endif
