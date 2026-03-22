/**
 ******************************************************************************
 * @file           : alg_crc.cpp
 * @brief          : CRC8和CRC16校验算法实现
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "alg_crc.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/**
 * @brief 计算CRC8校验值
 */
uint8_t CRCCalculator::calculateCRC8(const uint8_t *data, uint32_t length, uint8_t crc)
{
    while (length--) {
        crc = CRC8_TABLE[crc ^ (*data++)];
    }
    return crc;
}

/**
 * @brief 校验CRC8
 */
bool CRCCalculator::verifyCRC8(const uint8_t *data, uint32_t length)
{
    if (data == nullptr || length <= 1) return false;
    return calculateCRC8(data, length - 1) == data[length - 1];
}

/**
 * @brief 在数据末尾追加CRC8校验字节
 */
void CRCCalculator::appendCRC8(uint8_t *data, uint32_t length)
{
    if (data == nullptr || length <= 1) return;
    data[length - 1] = calculateCRC8(data, length - 1);
}

/**
 * @brief 计算CRC16校验值
 */
uint16_t CRCCalculator::calculateCRC16(const uint8_t *data, uint32_t length, uint16_t crc)
{
    if (data == nullptr) return 0xFFFF;
    while (length--) {
        uint8_t byte = *data++;
        crc          = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0x00FF];
    }
    return crc;
}

/**
 * @brief 校验CRC16
 */
bool CRCCalculator::verifyCRC16(const uint8_t *data, uint32_t length)
{
    if (data == nullptr || length <= 2) return false;
    uint16_t expected = calculateCRC16(data, length - 2);
    return (expected & 0xFF) == data[length - 2] &&
           ((expected >> 8) & 0xFF) == data[length - 1];
}

/**
 * @brief 在数据末尾追加CRC16校验字节（小端序）
 */
void CRCCalculator::appendCRC16(uint8_t *data, uint32_t length)
{
    if (data == nullptr || length <= 2) return;
    uint16_t crc     = calculateCRC16(data, length - 2);
    data[length - 2] = crc & 0xFF;
    data[length - 1] = (crc >> 8) & 0xFF;
}
