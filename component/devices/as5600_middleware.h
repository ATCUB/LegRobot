/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AS5600driver_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of AS5600.
  *             ���ļ���Ҫ�ṩI2C ��д��������ΪAS5600�������м��
  * @note       AS5600 only support I2C. AS5600ֻ֧��I2C��
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
#ifndef AS5600DRIVER_MIDDLEWARE_H
#define AS5600DRIVER_MIDDLEWARE_H

#include "struct_typedef.h"

#define AS5600_IIC_ADDRESS 0x36  //the I2C address of AS5600


/**
  * @brief          initialize AS5600 gpio.
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʼ��AS5600��GPIO
  * @param[in]      none
  * @retval         none
  */
extern void as5600_GPIO_init(void);

/**
  * @brief          initialize AS5600 communication interface
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʼ��AS5600��ͨ�Žӿ�
  * @param[in]      none
  * @retval         none
  */
extern void as5600_com_init(void);


/**
  * @brief          read a byte of AS5600 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          ��ȡAS5600��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
  */
extern uint8_t as5600_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          write a byte of AS5600 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          ͨ��I2Cд��һ���ֽڵ�AS5600�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
  * @retval         none
  */
extern void as5600_IIC_write_single_reg(uint8_t reg, uint8_t data);

/**
  * @brief          read multiple byte of AS5600 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          ��ȡAS5600�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void as5600_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * @brief          write multiple byte of AS5600 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          д�����ֽڵ�AS5600�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void as5600_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          ��ʱx����
  * @param[in]      ms: ms����
  * @retval         none
  */
extern void as5600_delay_ms(uint16_t ms);
/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          ��ʱx΢��
  * @param[in]      us: us΢��
  * @retval         none
  */
extern void as5600_delay_us(uint16_t us);
/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */


#endif
