/**
 ******************************************************************************
 * @file           : dvc_supercapacitor.hpp
 * @brief          : header file for dvc_supercapacitor.cpp
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
#include "drv_can.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 超级电容类
 * @details 实现主控板与超级电容控制器之间的CAN通信接口
 * @note 适用于26版超级电容控制器
 *       超级电容控制器CAN通信协议:
 *       主控 -> 超电: enableDCDC, systemRestart, refereePowerLimit, energyBuffer
 *       超电 -> 主控: errorCode, chassisPower, chassisPowerLimit, capEnergy
 */
class SuperCapacitor
{
public:
    /**
     * @brief 超级电容控制器错误码
     */
    enum ErrorCode : uint8_t {
        NO_ERROR        = 0x00,
        OUTPUT_DISABLED = 0x80, // bit7: 输出未使能
    };

private:
    // CAN通信
    uint32_t m_txCanID;             // 主控发给超电的CAN ID
    uint32_t m_rxCanID;             // 超电回报主控的CAN ID
    CAN_TxHeaderTypeDef m_txHeader; // CAN发送消息头
    uint8_t m_txData[8];            // CAN发送数据
    uint8_t m_rxData[8];            // CAN接收原始数据

    // 发送数据 (主控 -> 超电)
    struct TxData {
        uint8_t enableDCDC : 1;
        uint8_t systemRestart : 1;
        uint8_t resv0 : 6;
        uint16_t refereePowerLimit;
        uint16_t refereeEnergyBuffer;
        uint8_t resv1[3];
    } __attribute__((packed));

    // 接收数据 (超电 -> 主控)
    struct RxData {
        uint8_t errorCode;
        fp32 chassisPower;
        uint16_t chassisPowerLimit;
        uint8_t capEnergy;
    } __attribute__((packed));

    // 连接状态
    uint8_t m_feedbackSequence;
    uint8_t m_lastFeedbackSequence;
    uint8_t m_feedbackErrorCount;
    bool m_isConnected;
    bool m_pendingRestart;

public:
    SuperCapacitor(uint32_t txCanID = 0x061, uint32_t rxCanID = 0x051);

    // 通信相关
    uint32_t getTxCanID() const;
    uint32_t getRxCanID() const;
    const CAN_TxHeaderTypeDef *getControlHeader() const;
    const uint8_t *getControlData();
    bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
    bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
    const uint8_t *getFeedbackData() const;

    // 设置控制数据
    void setEnableDCDC(bool enable);
    void setSystemRestart();
    void setRefereePowerLimit(uint16_t powerLimit);
    void setRefereeEnergyBuffer(uint16_t energyBuffer);

    // 获取反馈数据
    uint8_t getErrorCode() const;
    bool isOutputEnabled() const;
    fp32 getChassisPower() const;
    uint16_t getChassisPowerLimit() const;
    fp32 getCapEnergyPercent() const;
    bool isConnected() const;

private:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage);
    inline void increaseFeedbackErrorCount();
    inline void clearFeedbackErrorCount();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/