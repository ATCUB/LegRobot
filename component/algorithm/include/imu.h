
#ifndef _IMU_H
#define _IMU_H

#include "main.h"

#define MPU6050_SENS_2G 			16.384f 0.00059814453125
#define MPU6050_SENS_4G 			8.192f
#define MPU6050_SENS_8G 			4.096f
#define MPU6050_SENS_16G 			2.048f




typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;

	float Pitch_v;
	float Roll_v;
	float Yaw_v;

	float ax;
	float ay;
	float az;

} IMU_Info;

IMU_Info *IMU_GetInfo(void);
uint8_t IMU_Init(void);
void IMU_Update(void);

#endif
