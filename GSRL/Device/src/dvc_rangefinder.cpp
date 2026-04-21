/**
 ******************************************************************************
 * @file           : dvc_rangefinder.cpp
 * @brief          : 测距传感器
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_rangefinder.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                          RangefinderTFMiniPlus实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化成员变量
 */
TFMiniPlus::TFMiniPlus()
    : m_rxDataPointer(nullptr),
      m_rxDataLength(0),
      m_lastDecodeTimestamp(0),
      m_distance(0),
      m_strength(0),
      m_temperature(0.0f),
      m_isConnected(false),
      m_isDataValid(false) {}

/**
 * @brief 在UART接收中断中调用，解码接收到的数据
 * @param data   DMA缓冲区指针
 * @param length 本次接收到的字节数
 */
void TFMiniPlus::receiveRxDataFromISR(const uint8_t *data, uint16_t length)
{
    m_rxDataPointer = data;
    m_rxDataLength  = length;
    decodeRxData();
}

/**
 * @brief 解析原始数据帧，更新距离、信号强度及传感器温度
 * @note 扫描缓冲区中最后一个合法帧；找到有效帧时更新时间戳和数据，否则直接返回
 */
void TFMiniPlus::decodeRxData()
{
    const uint8_t *pFrame = nullptr;
    if (m_rxDataPointer != nullptr && m_rxDataLength >= FRAME_LENGTH) {
        for (uint16_t i = 0; i <= m_rxDataLength - FRAME_LENGTH; i++) {
            // 帧头校验
            if (m_rxDataPointer[i] != FRAME_HEADER || m_rxDataPointer[i + 1] != FRAME_HEADER) {
                continue;
            }

            // 校验和校验: 前8字节累加和的低8位
            uint8_t checksum = 0;
            for (uint8_t j = 0; j < FRAME_LENGTH - 1; j++) {
                checksum += m_rxDataPointer[i + j];
            }
            if (checksum == m_rxDataPointer[i + FRAME_LENGTH - 1]) {
                pFrame = m_rxDataPointer + i;
            }
        }
    }

    if (pFrame == nullptr) { return; }

    // 解析数据
    m_distance       = (int16_t)((uint16_t)pFrame[2] | ((uint16_t)pFrame[3] << 8));
    m_strength       = (uint16_t)((uint16_t)pFrame[4] | ((uint16_t)pFrame[5] << 8));
    uint16_t tempRaw = (uint16_t)((uint16_t)pFrame[6] | ((uint16_t)pFrame[7] << 8));
    m_temperature    = static_cast<fp32>(tempRaw) / 8.0f - 256.0f;

    // 数据有效性判断: 信号强度 >= 100 且不等于 65535(过曝)
    m_isDataValid = (m_strength >= 100 && m_strength != 65535);

    // 更新连接状态和时间戳
    m_lastDecodeTimestamp = HAL_GetTick();
    m_isConnected         = true;
}

/**
 * @brief 获取测距结果
 * @return int16_t 距离值(cm)，数据无效时返回0
 */
int16_t TFMiniPlus::getDistance()
{
    isConnected();
    return m_isDataValid ? m_distance : 0;
}

/**
 * @brief 获取信号强度
 * @return uint16_t 信号强度，范围0~65535
 */
uint16_t TFMiniPlus::getStrength()
{
    isConnected();
    return m_strength;
}

/**
 * @brief 获取传感器连接状态，超时未收到数据则认为断开连接
 * @return bool 连接状态
 */
bool TFMiniPlus::isConnected()
{
    if (HAL_GetTick() - m_lastDecodeTimestamp > RX_TIMEOUT_MS) {
        m_distance    = 0;
        m_strength    = 0;
        m_temperature = 0.0f;
        m_isConnected = false;
        m_isDataValid = false;
    }
    return m_isConnected;
}

/**
 * @brief 获取芯片内部温度
 * @return fp32 温度值(℃)
 */
fp32 TFMiniPlus::getTemperature()
{
    isConnected();
    return m_temperature;
}

/**
 * @brief 获取以米为单位的测距结果
 * @return fp32 距离值(m)，数据无效时返回0
 */
fp32 TFMiniPlus::getDistanceM()
{
    return static_cast<fp32>(getDistance()) / 100.0f;
}

/**
 * @brief 获取数据有效性，信号强度不足或过曝时数据不可信
 * @return bool 数据有效性，Strength >= 100 且 Strength != 65535 时为true
 */
bool TFMiniPlus::isDataValid()
{
    isConnected();
    return m_isDataValid;
}
