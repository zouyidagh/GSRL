/**
 ******************************************************************************
 * @file           : drv_spi.c
 * @brief          : SPI驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_spi.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
SPI_Manage_Object_t s_spi_manage_objects[6] = {0}; // SPI管理对象

/**
 * @brief SPI管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的SPI实例
 */
static SPI_TypeDef *const spiInstances[6] = {
#ifdef USE_SPI1
    SPI1,
#endif
#ifdef USE_SPI2
    SPI2,
#endif
#ifdef USE_SPI3
    SPI3,
#endif
#ifdef USE_SPI4
    SPI4,
#endif
#ifdef USE_SPI5
    SPI5,
#endif
#ifdef USE_SPI6
    SPI6
#endif
};

/* User code -----------------------------------------------------------------*/

/**
 * @brief 获取SPI对象
 * @param hspi SPI句柄
 * @return SPI管理对象指针
 */
static SPI_Manage_Object_t *SPI_Get_Object(SPI_HandleTypeDef *hspi)
{
    for (int i = 0; i < 6; i++) {
        if (hspi->Instance == spiInstances[i]) {
            return &s_spi_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化SPI
 * @param hspi SPI句柄
 * @param rxCallbackFunction 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back rxCallbackFunction)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->hspi               = hspi;
        spi_obj->rxCallbackFunction = rxCallbackFunction;
    }
}

/**
 * @brief 切换SPI回调函数
 * @param hspi SPI句柄
 * @param rxCallbackFunction 处理回调函数
 */
void SPI_Switch_Callback_Function(SPI_HandleTypeDef *hspi, SPI_Call_Back rxCallbackFunction)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->rxCallbackFunction = rxCallbackFunction;
    }
}

HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t txDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        return HAL_SPI_Transmit_IT(hspi, pTxData, txDataLength);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t rxDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->exchangeDataLength = rxDataLength;
        return HAL_SPI_Receive_IT(hspi, spi_obj->rxBuffer, rxDataLength);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t exchangeDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->exchangeDataLength = exchangeDataLength;
        return HAL_SPI_TransmitReceive_IT(hspi, pTxData, spi_obj->rxBuffer, exchangeDataLength);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t txDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        return HAL_SPI_Transmit_DMA(hspi, pTxData, txDataLength);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint16_t rxDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->exchangeDataLength = rxDataLength;
        return HAL_SPI_Receive_DMA(hspi, spi_obj->rxBuffer, rxDataLength);
    }
    return HAL_ERROR;
}

/**
 * @brief 使用DMA方式发送并接收数据
 * @param hspi SPI句柄
 * @param pTxData 发送数据指针
 * @param exchangeDataLength 交换数据长度，单位字节
 * @retval HAL status
 */
HAL_StatusTypeDef SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t exchangeDataLength)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        spi_obj->exchangeDataLength = exchangeDataLength;
        return HAL_SPI_TransmitReceive_DMA(hspi, pTxData, spi_obj->rxBuffer, exchangeDataLength);
    }
    return HAL_ERROR;
}

/**
 * @brief HAL库SPI通信完成回调函数
 * @param hspi SPI句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        if (spi_obj->rxCallbackFunction != NULL) {
            spi_obj->rxCallbackFunction(spi_obj->rxBuffer, spi_obj->exchangeDataLength);
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Manage_Object_t *spi_obj = SPI_Get_Object(hspi);
    if (spi_obj != NULL) {
        if (spi_obj->rxCallbackFunction != NULL) {
            spi_obj->rxCallbackFunction(NULL, 0);
        }
    }
}
