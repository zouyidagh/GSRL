/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.cpp
 * @brief          : 大疆dr16遥控器接收器
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_remotecontrol.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                           Dr16RemoteControl类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化遥控器接收数据地址指针和连接状态
 */
Dr16RemoteControl::Dr16RemoteControl()
    : m_originalRxDataPointer(nullptr),
      m_isDr16Connected(false) {}

/**
 * @brief 从中断中获取DR16遥控器接收数据地址，更新相关标志位
 * @param data DR16遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */
void Dr16RemoteControl::receiveDr16RxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (DR16OriginalUARTRxData *)data;
    m_uartRxTimestamp       = HAL_GetTick(); // 更新接收时间戳
    m_isDr16Connected       = true;          // 接收到数据则认为遥控器连接正常
    m_isDecodeCompleted     = false;         // 标志解码未完成
}

/**
 * @brief 接收DR16遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */
void Dr16RemoteControl::decodeDr16RxData()
{
    // 判断遥控器连接状态，若使用的数据过时超过100ms则认为遥控器断开
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isDr16Connected = false;
        return;
    }
    // 解码遥控器数据, 每次接收数据仅解码一次
    if (m_isDecodeCompleted) return;
    m_rightStickX       = (fp32)(m_originalRxDataPointer->Channel_0 - 1024) / 660.0f;
    m_rightStickY       = (fp32)(m_originalRxDataPointer->Channel_1 - 1024) / 660.0f;
    m_leftStickX        = (fp32)(m_originalRxDataPointer->Channel_2 - 1024) / 660.0f;
    m_leftStickY        = (fp32)(m_originalRxDataPointer->Channel_3 - 1024) / 660.0f;
    m_scrollWheel       = (fp32)(m_originalRxDataPointer->Channel_4 - 1024) / 660.0f;
    m_mouseXSpeed       = (fp32)(m_originalRxDataPointer->Mouse_X / 32768.0f);
    m_mouseYSpeed       = (fp32)(m_originalRxDataPointer->Mouse_Y / 32768.0f);
    m_mouseWheelSpeed   = (fp32)(m_originalRxDataPointer->Mouse_Z / 32768.0f);
    m_isDecodeCompleted = true; // 更新解码完成标志，避免重复解码
}
