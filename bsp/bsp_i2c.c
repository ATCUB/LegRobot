
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
//	/* 恢复I2C寄存器为默认值 */
//	HAL_I2C_DeInit(&hi2c2); 
//	/* 重新初始化I2C外设 */
//	MX_I2C2_Init();
//	HAL_I2C_MspInit(&hi2c2);
//}

/**
  * @brief  写寄存器，这是提供给上层的接口
	* @param  slave_addr: 从机地址
	* @param 	reg_addr:寄存器地址
	* @param len：写入的长度
	*	@param data_ptr:指向要写入的数据
  * @retval 正常为0，不正常为非0
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                                        unsigned char reg_addr,
                                        unsigned short len, 
                                        unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c2, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT,data_ptr, len,I2Cx_FLAG_TIMEOUT); 
	/* 检查通讯状态 */
	if(status != HAL_OK)
	{
		/* 总线出错处理 */
		//I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	/* 检查SENSOR是否就绪进行下一次读写操作 */
	while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* 等待传输结束 */
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

/**
  * @brief  读寄存器，这是提供给上层的接口
	* @param  slave_addr: 从机地址
	* @param 	reg_addr:寄存器地址
	* @param len：要读取的长度
	*	@param data_ptr:指向要存储数据的指针
  * @retval 正常为0，不正常为非0
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                                       unsigned char reg_addr,
                                       unsigned short len, 
                                       unsigned char *data_ptr)
{
	HAL_StatusTypeDef status = HAL_OK;
	status =HAL_I2C_Mem_Read(&hi2c2,slave_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,data_ptr,len,I2Cx_FLAG_TIMEOUT);    
	/* 检查通讯状态 */
	if(status != HAL_OK)
	{
		/* 总线出错处理 */
		//I2Cx_Error(slave_addr);
	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	/* 检查SENSOR是否就绪进行下一次读写操作 */
	while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
	/* 等待传输结束 */
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}
