/**
 ******************************************************************************
 * @file           : drv_uart.h
 * @brief          : header file for drv_uart.c
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_UART_H
#define __DRV_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "usart.h"

/* Exported macro ------------------------------------------------------------*/
#define UART_BUFFER_SIZE 512 // UART接收缓冲区最大长度

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收处理回调函数类型定义
 * @param pRxData 接收数据指针
 * @param rxDataLength 接收数据长度
 */
typedef void (*UART_Call_Back)(uint8_t *pRxData, uint16_t rxDataLength);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    uint16_t rxDataLimit;
    UART_Call_Back rxCallbackFunction;
} UART_Manage_Object_t;

/* Exported variables --------------------------------------------------------*/
extern UART_Manage_Object_t s_uart_manage_objects[8]; // UART管理对象

/* Exported functions prototypes ---------------------------------------------*/
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit);
void UART_Reinit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength);

/* Defines -------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_UART_H */
