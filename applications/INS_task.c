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
	
#include "INS_task.h"
#include "mpu6050.h"



///**
//  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
//  *                 different install derection.
//  * @param[out]     gyro: after plus zero drift and rotate
//  * @param[out]     accel: after plus zero drift and rotate
//  * @param[out]     mag: after plus zero drift and rotate
//  * @param[in]      bmi088: gyro and accel data
//  * @param[in]      ist8310: mag data
//  * @retval         none
//  */
///**
//  * @brief          旋转陀螺仪,加速度计,并计算零漂,因为设备有不同安装方式
//  * @param[out]     gyro: 加上零漂和旋转
//  * @param[out]     accel: 加上零漂和旋转
//  * @param[in]      mpu6050: 陀螺仪和加速度计数据
//  * @retval         none
//  */
//static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], mpu6050_real_data_t *mpu6050);


//mpu6050_real_data_t  mpu6050_real_data;
//fp32 gyro_scale_factor[3][3] = {MPU6050_BOARD_INSTALL_SPIN_MATRIX};
//fp32 gyro_offset[3];
//fp32 gyro_cali_offset[3];

//fp32 accel_scale_factor[3][3] = {MPU6050_BOARD_INSTALL_SPIN_MATRIX};
//fp32 accel_offset[3];
//fp32 accel_cali_offset[3];


//static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

////加速度计低通滤波
//static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
//static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
//static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


//static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
//static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
//static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
//static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
//fp32 INS_angle_system[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad


/**
  * @brief          imu task, init mpu6050, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 mpu6050, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

//void INS_task(void)
//{
//	
//		//wait a time
//		HAL_Delay(INS_TASK_INIT_TIME);
//		while(MPU6050_Init())
//		{
//				HAL_Delay(100);
//		}

//		//MPU6050_read(mpu6050_real_data.gyro, mpu6050_real_data.accel, &mpu6050_real_data.temp);

////		AHRS_init(INS_quat, INS_accel, INS_mag);
//		
//		accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
//    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
//    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
//		
//		while(1)
//		{
//			
//				//MPU6050_read(mpu6050_real_data.gyro, mpu6050_real_data.accel, &mpu6050_real_data.temp);
//				HAL_Delay(1);
//				imu_cali_slove(INS_gyro, INS_accel, &mpu6050_real_data);
//				
//				//加速度计低通滤波
//        //accel low-pass filter
//        accel_fliter_1[0] = accel_fliter_2[0];
//        accel_fliter_2[0] = accel_fliter_3[0];

//        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

//        accel_fliter_1[1] = accel_fliter_2[1];
//        accel_fliter_2[1] = accel_fliter_3[1];

//        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

//        accel_fliter_1[2] = accel_fliter_2[2];
//        accel_fliter_2[2] = accel_fliter_3[2];

//        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
//				
////				AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
////        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
//				
//				for(uint8_t i=0; i < 3; i++)
//				{
//					INS_angle_system[i] = INS_angle[i] * 57.3f;
//				}
//				
//				HAL_Delay(20);
//		}
//}

/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have 
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[in]      mpu6050: 陀螺仪和加速度计数据
  * @retval         none
  */
//static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], mpu6050_real_data_t *mpu6050)
//{
//    for (uint8_t i = 0; i < 3; i++)
//    {
//        gyro[i] = mpu6050->gyro[0] * gyro_scale_factor[i][0] + mpu6050->gyro[1] * gyro_scale_factor[i][1] + mpu6050->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
//        accel[i] = mpu6050->accel[0] * accel_scale_factor[i][0] + mpu6050->accel[1] * accel_scale_factor[i][1] + mpu6050->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
//    }
//		
//		
//}








