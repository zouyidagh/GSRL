/**
 ******************************************************************************
 * @file           : drv_spi.h
 * @brief          : header file for drv_spi.c
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "spi.h"

/* Exported macro ------------------------------------------------------------*/
#define SPI_BUFFER_SIZE 256 // SPI接收缓冲区最大长度

/* Exported types ------------------------------------------------------------*/

/**
 * @brief SPI通信接收回调函数数据类型
 * @param pRxData 接收数据指针
 * @param rxDataLength 接收数据长度
 */
typedef void (*SPI_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength);

/**
 * @brief SPI通信处理结构体
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;
    uint8_t rxBuffer[SPI_BUFFER_SIZE];
    uint16_t exchangeDataLength;
    SPI_Call_Back rxCallbackFunction;
} SPI_Manage_Object_t;

/* Exported variables --------------------------------------------------------*/
extern SPI_Manage_Object_t s_spi_manage_objects[6]; // SPI管理对象

/* Exported functions prototypes ---------------------------------------------*/
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Call_Back rxCallbackFunction);
void SPI_Switch_Callback_Function(SPI_HandleTypeDef *hspi, SPI_Call_Back rxCallbackFunction);
HAL_StatusTypeDef SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t txDataLength);
HAL_StatusTypeDef SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint16_t rxDataLength);
HAL_StatusTypeDef SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t exchangeDataLength);
HAL_StatusTypeDef SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t txDataLength);
HAL_StatusTypeDef SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint16_t rxDataLength);
HAL_StatusTypeDef SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t exchangeDataLength);

/* Defines -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_SPI_H */
