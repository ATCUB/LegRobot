
#include "bsp_i2c.h"
#include "i2c.h"
#include "usart.h"

///**
//  * @brief  Manages error callback by re-initializing I2C.
//  * @param  Addr: I2C Address
//  * @retval None
//  */
//static void I2Cx_Error(uint8_t Addr)
//{
//	/* �ָ�I2C�Ĵ���ΪĬ��ֵ */
//	HAL_I2C_DeInit(&hi2c2); 
//	/* ���³�ʼ��I2C���� */
//	MX_I2C2_Init();
//	HAL_I2C_MspInit(&hi2c2);
//}

/**
  * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
	* @param  slave_addr: �ӻ���ַ
	* @param 	reg_addr:�Ĵ�����ַ
	* @param len��д��ĳ���
	*	@param data_ptr:ָ��Ҫд�������
  * @retval ����Ϊ0��������Ϊ��0
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c2, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,data_ptr, len,I2Cx_FLAG_TIMEOUT); 
	/* ���ͨѶ״̬ */
	if(status != HAL_OK)
	{
		/* ���߳����� */
		//I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

/**
  * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
	* @param  slave_addr: �ӻ���ַ
	* @param 	reg_addr:�Ĵ�����ַ
	* @param len��Ҫ��ȡ�ĳ���
	*	@param data_ptr:ָ��Ҫ�洢���ݵ�ָ��
  * @retval ����Ϊ0��������Ϊ��0
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status =HAL_I2C_Mem_Read(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,data_ptr,len,I2Cx_FLAG_TIMEOUT);    
	/* ���ͨѶ״̬ */
	if(status != HAL_OK)
	{
		/* ���߳����� */
		//I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	/* ���SENSOR�Ƿ����������һ�ζ�д���� */
	while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* �ȴ�������� */
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}
