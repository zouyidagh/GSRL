/**
 ******************************************************************************
 * @file           : dvc_supercapacitor.cpp
 * @brief          : 超级电容通信模块
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_supercapacitor.hpp"
#include <cstring>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/**
 * @brief 超级电容类构造函数
 * @param txCanID 主控发给超电的CAN ID, 默认0x061
 * @param rxCanID 超电回报主控的CAN ID, 默认0x051
 */
SuperCapacitor::SuperCapacitor(uint32_t txCanID, uint32_t rxCanID)
    : m_txCanID(txCanID),
      m_rxCanID(rxCanID),
      m_feedbackSequence(0),
      m_lastFeedbackSequence(0),
      m_feedbackErrorCount(0),
      m_isConnected(false),
      m_pendingRestart(false)
{
    static_assert(sizeof(TxData) == 8, "TxData size error");
    static_assert(sizeof(RxData) == 8, "RxData size error");

    m_txHeader.StdId              = m_txCanID;
    m_txHeader.IDE                = CAN_ID_STD;
    m_txHeader.RTR                = CAN_RTR_DATA;
    m_txHeader.DLC                = 8;
    m_txHeader.TransmitGlobalTime = DISABLE;

    memset(m_txData, 0, sizeof(m_txData));
    memset(m_rxData, 0, sizeof(m_rxData));
}

/**
 * @brief 获取发送CAN ID
 * @return uint32_t 主控发给超电的CAN ID
 */
uint32_t SuperCapacitor::getTxCanID() const
{
    return m_txCanID;
}

/**
 * @brief 获取反馈CAN ID
 * @return uint32_t 超电回报主控的CAN ID
 */
uint32_t SuperCapacitor::getRxCanID() const
{
    return m_rxCanID;
}

/**
 * @brief 获取CAN发送消息头结构体指针
 * @return const CAN_TxHeaderTypeDef* CAN发送消息头指针
 */
const CAN_TxHeaderTypeDef *SuperCapacitor::getControlHeader() const
{
    return &m_txHeader;
}

/**
 * @brief 获取CAN发送数据, 同时检测超电连接状态
 * @return const uint8_t* CAN发送数据m_txData[8]
 */
const uint8_t *SuperCapacitor::getControlData()
{
    // 检测连接状态
    if (m_feedbackSequence == m_lastFeedbackSequence) {
        increaseFeedbackErrorCount();
    } else {
        m_lastFeedbackSequence = m_feedbackSequence; // 滚动更新反馈数据序号
    }

    // 若标志置位则发送一次系统重启指令
    TxData *tx        = reinterpret_cast<TxData *>(m_txData);
    tx->systemRestart = m_pendingRestart ? 1 : 0;
    m_pendingRestart  = false;

    return m_txData;
}

/**
 * @brief 从CAN接收缓冲数组中从后往前解析超电反馈数据
 * @param rxMessage CAN接收缓冲数组
 * @param Size CAN接收缓冲数组大小
 * @return bool 解析成功返回true，否则返回false
 * @note 从后往前解析, 缓冲数组中数据应从旧到新排列
 */
bool SuperCapacitor::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size)
{
    for (uint8_t i = Size; i > 0; i--) {
        if (decodeCanRxMessage(rxMessage[i - 1])) {
            clearFeedbackErrorCount();
            return true;
        }
    }
    increaseFeedbackErrorCount(); // 未解析到数据增加错误计数
    return false;
}

/**
 * @brief 从中断服务函数中解析超电反馈数据
 * @param rxMessage CAN接收消息
 * @return bool 解析成功返回true，否则返回false
 * @note 该函数仅供CAN中断回调函数调用
 * @note 返回false可能是ID不匹配，属正常情况
 */
bool SuperCapacitor::decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage)
{
    if (decodeCanRxMessage(*rxMessage)) {
        m_feedbackSequence++;
        clearFeedbackErrorCount();
        return true;
    }
    return false;
}

/**
 * @brief 获取CAN反馈原始数据
 * @return const uint8_t* CAN反馈原始数据m_rxData[8]
 */
const uint8_t *SuperCapacitor::getFeedbackData() const
{
    return m_rxData;
}

/**
 * @brief 解析CAN接收消息
 * @param rxMessage CAN接收消息
 * @return bool 解析成功返回true，否则返回false
 */
bool SuperCapacitor::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_rxCanID || rxMessage.header.DLC != 8 || rxMessage.header.IDE != CAN_ID_STD) {
        return false;
    }

    memcpy(m_rxData, rxMessage.data, sizeof(m_rxData));
    return true;
}

/**
 * @brief 设置DCDC使能
 * @param enable true: 使能DCDC, false: 关闭DCDC
 */
void SuperCapacitor::setEnableDCDC(bool enable)
{
    TxData *tx     = reinterpret_cast<TxData *>(m_txData);
    tx->enableDCDC = enable ? 1 : 0;
}

/**
 * @brief 发送系统重启指令
 * @note 发送后自动清除重启标志
 */
void SuperCapacitor::setSystemRestart()
{
    m_pendingRestart = true;
}

/**
 * @brief 设置裁判系统功率限制
 * @param powerLimit 功率限制值 单位W
 */
void SuperCapacitor::setRefereePowerLimit(uint16_t powerLimit)
{
    TxData *tx                    = reinterpret_cast<TxData *>(m_txData);
    tx->refereePowerLimit = powerLimit;
}

/**
 * @brief 设置裁判系统能量缓冲
 * @param energyBuffer 能量缓冲值 单位J
 */
void SuperCapacitor::setRefereeEnergyBuffer(uint16_t energyBuffer)
{
    TxData *tx                      = reinterpret_cast<TxData *>(m_txData);
    tx->refereeEnergyBuffer = energyBuffer;
}

/**
 * @brief 获取超级电容控制器错误码
 * @return uint8_t 错误码
 */
uint8_t SuperCapacitor::getErrorCode() const
{
    const RxData *rx = reinterpret_cast<const RxData *>(m_rxData);
    return rx->errorCode;
}

/**
 * @brief 检查超级电容输出是否使能
 * @return true 输出已使能
 * @return false 输出未使能
 */
bool SuperCapacitor::isOutputEnabled() const
{
    const RxData *rx = reinterpret_cast<const RxData *>(m_rxData);
    return !(rx->errorCode & OUTPUT_DISABLED);
}

/**
 * @brief 获取底盘功率
 * @return fp32 底盘功率 单位W
 */
fp32 SuperCapacitor::getChassisPower() const
{
    const RxData *rx = reinterpret_cast<const RxData *>(m_rxData);
    return rx->chassisPower;
}

/**
 * @brief 获取底盘功率限制
 * @return uint16_t 底盘功率限制 单位W
 */
uint16_t SuperCapacitor::getChassisPowerLimit() const
{
    const RxData *rx = reinterpret_cast<const RxData *>(m_rxData);
    return rx->chassisPowerLimit;
}

/**
 * @brief 获取电容能量百分比
 * @return fp32 电容能量百分比 [0.0f, 1.0f]
 */
fp32 SuperCapacitor::getCapEnergyPercent() const
{
    const RxData *rx = reinterpret_cast<const RxData *>(m_rxData);
    return (fp32)rx->capEnergy / 255.0f;
}

/**
 * @brief 检查超级电容是否连接
 * @return true 超级电容连接正常
 * @return false 超级电容连接异常
 */
bool SuperCapacitor::isConnected() const
{
    return m_isConnected;
}

/**
 * @brief 增加反馈错误计数, 同时更新连接状态
 * @note 反馈错误计数超过10则认为超电连接异常
 */
inline void SuperCapacitor::increaseFeedbackErrorCount()
{
    if (m_feedbackErrorCount < 255) {
        m_feedbackErrorCount++;
    }
    m_isConnected = m_feedbackErrorCount < 10;
}

/**
 * @brief 清除反馈错误计数, 同时更新连接状态
 */
inline void SuperCapacitor::clearFeedbackErrorCount()
{
    m_feedbackErrorCount = 0;
    m_isConnected        = true;
}
