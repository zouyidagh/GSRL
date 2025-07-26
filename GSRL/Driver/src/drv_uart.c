/**
 ******************************************************************************
 * @file           : drv_uart.c
 * @brief          : UART驱动
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "drv_uart.h"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
UART_Manage_Object_t s_uart_manage_objects[8] = {0}; // UART管理对象

/**
 * @brief UART管理实例数组
 * @note 根据board_config.h中的宏定义配置决定使用的UART实例
 */
static USART_TypeDef *const uartInstances[8] = {
#ifdef USE_USART1
    USART1,
#endif
#ifdef USE_USART2
    USART2,
#endif
#ifdef USE_USART3
    USART3,
#endif
#ifdef USE_UART4
    UART4,
#endif
#ifdef USE_UART5
    UART5,
#endif
#ifdef USE_USART6
    USART6,
#endif
#ifdef USE_UART7
    UART7,
#endif
#ifdef USE_UART8
    UART8
#endif
};

/* User code -----------------------------------------------------------------*/

/**
 * @brief 获取UART对象
 * @param huart UART句柄
 * @return UART管理对象指针
 */
static UART_Manage_Object_t *UART_Get_Object(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < 8; i++) {
        if (huart->Instance == uartInstances[i]) {
            return &s_uart_manage_objects[i];
        }
    }
    return NULL;
}

/**
 * @brief 初始化UART，启动DMA接收
 * @param huart UART句柄
 * @param rxCallbackFunction 处理回调函数
 * @param rxDataLimit 空闲中断最大接收数据长度，不得超过drv_uart.h中定义的缓冲区大小UART_BUFFER_SIZE
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back rxCallbackFunction, uint16_t rxDataLimit)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        uart_obj->huart              = huart;
        uart_obj->rxDataLimit        = rxDataLimit;
        uart_obj->rxCallbackFunction = rxCallbackFunction;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief 重新初始化UART并重启DMA接收
 * @param huart UART句柄
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief 发送数据帧
 * @param huart UART句柄
 * @param pTxData 被发送的数据指针
 * @param txDataLength 发送数据长度
 * @retval HAL status
 */
HAL_StatusTypeDef UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *pTxData, uint16_t txDataLength)
{
    return (HAL_UART_Transmit_DMA(huart, pTxData, txDataLength));
}

/**
 * @brief HAL库UART接收空闲中断
 * @param huart UART编号
 * @param Size 长度
 * @details 判断并调用对应的回调处理函数后重启DMA接收
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        if (uart_obj->rxCallbackFunction != NULL) {
            uart_obj->rxCallbackFunction(uart_obj->rxBuffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
    }
}

/**
 * @brief HAL库UART接收错误中断
 * @param huart UART编号
 * @details 发生错误重启DMA接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Manage_Object_t *uart_obj = UART_Get_Object(huart);
    if (uart_obj != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->rxBuffer, uart_obj->rxDataLimit);
        if (uart_obj->rxCallbackFunction != NULL) {
            uart_obj->rxCallbackFunction(NULL, 0);
        }
    }
}
