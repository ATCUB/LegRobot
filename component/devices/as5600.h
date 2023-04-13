
/**
  * @file       AS5600driver.c/h
  * @brief      
  * @note       AS5600 only support I2C
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  */

#ifndef AS5600DRIVER_H
#define AS5600DRIVER_H
#include "struct_typedef.h"

#define AS5600_DATA_READY_BIT 2


#define	_raw_ang_hi 0x0C
#define	_raw_ang_lo 0x0D

#define AS5600_NO_ERROR 0x00
#define AS5600_NO_SENSOR 0x40


extern fp32 foc_rad;
extern fp32 foc_angle;

typedef struct AS5600_real_data_t
{
	fp32 angle;
  fp32 mag;
} AS5600_real_data_t;

typedef struct AS5600_raw_data_t
{
  uint8_t status;
  AS5600_real_data_t foc_mag_motor;
} AS5600_raw_data_t;

/**
  * @brief          initialize AS5600
  * @param[in]      none
  * @retval         error value
  */
/**
  * @brief          初始化AS5600
  * @param[in]      none
  * @retval         error value
  */ 
extern uint8_t AS5600_init(void);

/**
  * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve. 
  * @param[in]      status_buf:the data point from the STAT1(0x02) register of AS5600 to the DATAZL(0x08) register 
  * @param[out]     AS5600_real_data:AS5600 data struct 
  * @retval         none
  */
/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAZL的数据，可以使用这个函数进行处理
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAZL(0x08)寄存器 
  * @param[out]     AS5600_real_data:AS5600的数据结构
  * @retval         none
  */
extern void AS5600_read_over(uint8_t *status_buf, AS5600_real_data_t *AS5600_real_data);

/**
  * @brief          read mag magnetic field strength data of AS5600 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          通过读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
extern void as5600_read_mag(void);


extern const AS5600_raw_data_t *get_motor_mag_point(void);

#endif
