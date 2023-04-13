
/**
  * @file       AS5600driver.c/h
  * @brief      
  * @note       AS5600 only support I2C
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
  * @brief          ��ʼ��AS5600
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
  * @brief          ����Ѿ�ͨ��I2C��DMA��ʽ��ȡ���˴�STAT1��DATAZL�����ݣ�����ʹ������������д���
  * @param[in]      status_buf:����ָ��,��STAT1(0x02) �Ĵ����� DATAZL(0x08)�Ĵ��� 
  * @param[out]     AS5600_real_data:AS5600�����ݽṹ
  * @retval         none
  */
extern void AS5600_read_over(uint8_t *status_buf, AS5600_real_data_t *AS5600_real_data);

/**
  * @brief          read mag magnetic field strength data of AS5600 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          ͨ����ȡ�ų�����
  * @param[out]     �ų�����
  * @retval         none
  */
extern void as5600_read_mag(void);


extern const AS5600_raw_data_t *get_motor_mag_point(void);

#endif
