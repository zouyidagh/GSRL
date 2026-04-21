/**
 ******************************************************************************
 * @file           : dvc_rangefinder.hpp
 * @brief          : header file for dvc_rangefinder.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 测距传感器基类
 * @note 该类不可实例化，仅定义数据获取接口，与通信协议无关
 * @details 该类的派生类需实现以下方法：
 * - getDistance  获取距离值(cm)
 * - getStrength  获取信号强度
 * - isConnected  获取连接状态
 */
class Rangefinder
{
public:
    virtual ~Rangefinder() = default;

    virtual int16_t getDistance()  = 0; // cm，数据无效时返回0
    virtual uint16_t getStrength() = 0;
    virtual bool isConnected()     = 0;

protected:
    Rangefinder() = default;
};

/**
 * @brief 北醒 TFmini Plus 激光测距传感器类
 * @note 使用UART通信，波特率115200，8N1
 * @note 使用前需确保receiveRxDataFromISR方法在对应UART接收中断服务函数中被调用
 * @note 数据帧格式: [0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Checksum]
 */
class TFMiniPlus : public Rangefinder
{
public:
    static constexpr uint8_t FRAME_HEADER = 0x59;
    static constexpr uint8_t FRAME_LENGTH = 9;

private:
    static constexpr uint32_t RX_TIMEOUT_MS = 100;

    const uint8_t *m_rxDataPointer; // 指向UART DMA缓冲区的指针
    uint16_t m_rxDataLength;
    uint32_t m_lastDecodeTimestamp;

    int16_t m_distance;  // 距离值(cm)，信号无效时为0
    uint16_t m_strength; // 信号强度
    fp32 m_temperature;  // 芯片内部温度(℃)
    bool m_isConnected;
    bool m_isDataValid;  // 数据有效性: Strength >= 100 且 Strength != 65535

    void decodeRxData();

public:
    TFMiniPlus();

    void receiveRxDataFromISR(const uint8_t *data, uint16_t length);

    int16_t getDistance() override;  // cm，数据无效时返回0
    uint16_t getStrength() override;
    bool isConnected() override;
    fp32 getTemperature();           // ℃
    fp32 getDistanceM();             // 米
    bool isDataValid();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
