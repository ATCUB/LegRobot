
  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       as5600_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of as5600.
  *             本文件主要提供I2C 读写函数，作为as5600驱动的中间件
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

#include "as5600_middleware.h"
#include "main.h"

//extern I2C_HandleTypeDef hi2c1;

/**
  * @brief          initialize as5600 gpio.
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化as5600的GPIO
  * @param[in]      none
  * @retval         none
  */
void as5600_GPIO_init(void)
{

}

/**
  * @brief          initialize as5600 communication interface
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          初始化as5600的通信接口
  * @param[in]      none
  * @retval         none
  */
void as5600_com_init(void)
{
}

/**
  * @brief          read a byte of as5600 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          读取as5600的一个字节通过I2C
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t as5600_IIC_read_single_reg(uint8_t reg)
{
    uint8_t res = 0;
//    HAL_I2C_Mem_Read(&hi2c1, AS5600_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&res,1,10);
    return res;
}


/**
  * @brief          write a byte of as5600 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          通过I2C写入一个字节到as5600的寄存器中
  * @param[in]      寄存器地址
  * @param[in]      写入值
  * @retval         none
  */
void as5600_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
//    HAL_I2C_Mem_Write(&hi2c1, AS5600_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,&data,1,10);
}

/**
  * @brief          read multiple byte of as5600 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          读取as5600的多个字节通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void as5600_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
//    HAL_I2C_Mem_Read(&hi2c1, AS5600_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,buf,len,10);
}


/**
  * @brief          write multiple byte of as5600 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          写入多个字节到as5600的寄存器通过I2C
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节总数
  * @retval         none
  */
void as5600_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
//    HAL_I2C_Mem_Write(&hi2c1, AS5600_IIC_ADDRESS <<1, reg,I2C_MEMADD_SIZE_8BIT,data,len,10);
}

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
void as5600_delay_ms(uint16_t ms)
{
    HAL_Delay(ms);
}


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
void as5600_delay_us(uint16_t us)
{
    uint32_t ticks = 0;
    uint32_t told = 0, tnow = 0, tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 72;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}


