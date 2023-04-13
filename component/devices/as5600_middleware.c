
  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       as5600_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of as5600.
  *             ���ļ���Ҫ�ṩI2C ��д��������Ϊas5600�������м��
  * @note       as5600 only support I2C. as5600ֻ֧��I2C��
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
  * @brief          ��ʼ��as5600��GPIO
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
  * @brief          ��ʼ��as5600��ͨ�Žӿ�
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
  * @brief          ��ȡas5600��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
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
  * @brief          ͨ��I2Cд��һ���ֽڵ�as5600�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
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
  * @brief          ��ȡas5600�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
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
  * @brief          д�����ֽڵ�as5600�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
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
  * @brief          ��ʱx����
  * @param[in]      ms: ms����
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
  * @brief          ��ʱx΢��
  * @param[in]      us: us΢��
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


