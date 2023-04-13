/**
  ****************************(C) COPYRIGHT 2023 LY****************************
  * @file       INS_task.c/h
  * @brief      use mpu6050 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪mpu6050，完成姿态解算，得出欧拉角，
  *             未通过mpu6050的data ready 中断外部触发
  *             通过DMA的IIC传输节约CPU时间
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2023     LY              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2023 LY****************************
  */


#ifndef INS_Task_H
#define INS_Task_H

#include "struct_typedef.h"
#include "main.h"

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间


#define MPU6050_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}   

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

		
		



















#endif


