/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AS5600driver_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of AS5600.
  *             本文件主要提供I2C 读写函数，作为AS5600驱动的中间件
  * @note       AS5600 only support I2C. AS5600只支持I2C。
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
  * @brief          初始化AS5600的GPIO
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
  * @brief          初始化AS5600的通信接口
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
  * @brief          读取AS5600的一个字节通过I2C
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
extern uint8_t as5600_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          write a byte of AS5600 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          通过I2C写入一个字节到AS5600的寄存器中
  * @param[in]      寄存器地址
  * @param[in]      写入值
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
  * @brief          读取AS5600的多个字节通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
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
  * @brief          写入多个字节到AS5600的寄存器通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
extern void as5600_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          延时x毫秒
  * @param[in]      ms: ms毫秒
  * @retval         none
  */
extern void as5600_delay_ms(uint16_t ms);
/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          延时x微秒
  * @param[in]      us: us微秒
  * @retval         none
  */
extern void as5600_delay_us(uint16_t us);
/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */


#endif
