
  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       as5600driver.c/h
  * @brief      as5600 is a 3-axis digital magnetometer, the file includes initialization function,
  *             read magnetic field strength data function.
  *             as5600是一款磁力编码器，本文件包括初始化函数，读取位置数据函数。
  * @note       as5600 only support I2C. as5600只支持I2C。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "as5600.h"
#include "as5600_middleware.h"
#include "usart.h"

#define ANGLE2RAD 0.01745329251994329576923690768489f //raw int16 data change to uT unit. 原始整型数据变成 单位ut

#define as5600_WHO_AM_I 0x00       //as5600 "who am I " 
#define as5600_WHO_AM_I_VALUE 0x36 //device ID

#define as5600_WRITE_REG_NUM 4 

fp32 foc_rad = 0;
fp32 foc_angle = 0;
static AS5600_raw_data_t as5600_raw_mag_data[2];

//the first column:the registers of as5600. 第一列:as5600的寄存器
//the second column: the value to be writed to the registers.第二列:需要写入的寄存器值
//the third column: return error value.第三列:返回的错误码
static const uint8_t as5600_write_reg_data_error[as5600_WRITE_REG_NUM][3] ={
        {0x0B, 0x08, 0x01},     //enalbe interrupt  and low pin polarity.开启中断，并且设置低电平
        {0x41, 0x09, 0x02},     //average 2 times.平均采样两次
        {0x42, 0xC0, 0x03},     //must be 0xC0. 必须是0xC0
        {0x0A, 0x0B, 0x04}};    //200Hz output rate.200Hz输出频率



/**
  * @brief          initialize as5600
  * @param[in]      none
  * @retval         error value
  */
/**
  * @brief          初始化as5600
  * @param[in]      none
  * @retval         error value
  */
uint8_t as5600_init(void)
{
    static const uint8_t wait_time = 150;
    static const uint8_t sleepTime = 50;
    uint8_t res = 0;
    uint8_t writeNum = 0;

//    res = as5600_IIC_read_single_reg(as5600_WHO_AM_I);
//    if (res != as5600_WHO_AM_I_VALUE)
//    {
//        return AS5600_NO_SENSOR;
//    }

    //set as560000 sonsor config and check
    for (writeNum = 0; writeNum < as5600_WRITE_REG_NUM; writeNum++)
    {
        as5600_IIC_write_single_reg(as5600_write_reg_data_error[writeNum][0], as5600_write_reg_data_error[writeNum][1]);
        as5600_delay_us(wait_time);
        res = as5600_IIC_read_single_reg(as5600_write_reg_data_error[writeNum][0]);
        as5600_delay_us(wait_time);
        if (res != as5600_write_reg_data_error[writeNum][1])
        {
            return as5600_write_reg_data_error[writeNum][2];
        }
    }
    return AS5600_NO_ERROR;
}

/**
  * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve. 
  * @param[in]      status_buf:the data point from the STAT1(0x02) register of as5600 to the DATAZL(0x08) register 
  * @param[out]     as5600_real_data:as5600 data struct 
  * @retval         none
  */
/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAZL的数据，可以使用这个函数进行处理
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAZL(0x08)寄存器 
  * @param[out]     as5600_real_data:as5600的数据结构
  * @retval         none
  */
void as5600_read_over(uint8_t *status_buf, AS5600_real_data_t *as5600_real_data)
{

//    if (status_buf[0] & 0x01)
//    {
//        int16_t temp_as5600_data = 0;
//        as5600_raw_mag_data.status |= 1 << AS5600_DATA_READY_BIT;

////        temp_as5600_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
////        as5600_real_data->mag[0] = MAG_SEN * temp_as5600_data;
////        temp_as5600_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
////        as5600_real_data->mag[1] = MAG_SEN * temp_as5600_data;
////        temp_as5600_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
////        as5600_real_data->mag[2] = MAG_SEN * temp_as5600_data;
//    }
//    else
//    {
//        as5600_raw_mag_data.status &= ~(1 << AS5600_DATA_READY_BIT);
//    }
}

/**
  * @brief          read mag magnetic field strength data of as5600 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          通过读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
void as5600_read_mag(void)
{
    uint8_t buf[6];
    int16_t temp_as5600_data = 0;
    //read the "DATAXL" register (0x0C)
    as5600_IIC_read_muli_reg(0x0C, &buf[0], 1);
		as5600_IIC_read_muli_reg(0x0D, &buf[1], 1);
		
		temp_as5600_data = (buf[0] << 8) | buf[1];
		foc_rad = ((fp32)temp_as5600_data / 4096.0) * 360.0 * ANGLE2RAD;
		foc_angle = ((fp32)temp_as5600_data / 4096.0) * 360.0;
//		as5600_raw_mag_data[0].foc_mag_motor.mag = temp_as5600_data;
//		as5600_raw_mag_data[0].foc_mag_motor.angle = as5600_raw_mag_data[0].foc_mag_motor.mag / 4096 * 360 * ANGLE2RAD;
		
//    temp_as5600_data = (int16_t)((buf[1] << 8) | buf[0]);
//    mag[0] = MAG_SEN * temp_as5600_data;
//    temp_as5600_data = (int16_t)((buf[3] << 8) | buf[2]);
//    mag[1] = MAG_SEN * temp_as5600_data;
//    temp_as5600_data = (int16_t)((buf[5] << 8) | buf[4]);
//    mag[2] = MAG_SEN * temp_as5600_data;
}

/**
  * @brief          return yaw motor data point
  * @param[in]      none
  * @retval         yaw motor data point
  */
/**
  * @brief          返回电机磁场位置数据指针
  * @param[in]      none
  * @retval         电机位置数据指针
  */
const AS5600_raw_data_t *get_motor_mag_point(void)
{
    return &as5600_raw_mag_data[0];
}
