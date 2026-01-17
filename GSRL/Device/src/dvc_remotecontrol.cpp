/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.cpp
 * @brief          : 遥控器驱动
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
 *                          RemoteControl遥控器基类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化成员变量，设置摇杆死区
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 * @note 该构造函数不可被外部调用，需由派生类构造函数调用
 */
RemoteControl::RemoteControl(fp32 stickDeadZone)
    : m_uartRxTimestamp(0),
      m_isConnected(false),
      m_isDecodeCompleted(false),
      m_stickDeadZone(stickDeadZone) {}

RemoteControl::SwitchEvent2Pos RemoteControl::judgeSwitchEvent(SwitchStatus2Pos currentStatus, SwitchStatus2Pos lastStatus)
{
    // SwitchEvent2Pos枚举值:
    // SWITCH_UP     = 1
    // SWITCH_DOWN   = 2
    // SWITCH_ERROR  = -1
    if (currentStatus == SwitchStatus2Pos::SWITCH_ERROR || lastStatus == SwitchStatus2Pos::SWITCH_ERROR) {
        return SwitchEvent2Pos::SWITCH_EVENT_STATUS_ERROR;
    }
    // 使用枚举值差值判断跳变事件
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return SwitchEvent2Pos::SWITCH_NO_CHANGE;
        case -1:
            return SwitchEvent2Pos::SWITCH_TOGGLE_DOWN_UP;
        case 1:
            return SwitchEvent2Pos::SWITCH_TOGGLE_UP_DOWN;
        default:
            return SwitchEvent2Pos::SWITCH_EVENT_OUT_OF_RANGE_ERROR;
    }
}

/**
 * @brief 根据当前状态和上一次状态返回拨杆的跳变事件
 * @note 本函数为内部函数, 请勿在外部调用
 */
RemoteControl::SwitchEvent3Pos RemoteControl::judgeSwitchEvent(SwitchStatus3Pos currentStatus, SwitchStatus3Pos lastStatus)
{
    // SwitchEvent3Pos枚举值:
    // SWITCH_UP     = 1
    // SWITCH_DOWN   = 2
    // SWITCH_MIDDLE = 3
    // SWITCH_ERROR  = -1
    if (currentStatus == SwitchStatus3Pos::SWITCH_ERROR || lastStatus == SwitchStatus3Pos::SWITCH_ERROR) {
        return SwitchEvent3Pos::SWITCH_EVENT_STATUS_ERROR;
    }
    // 使用枚举值差值判断跳变事件
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return SwitchEvent3Pos::SWITCH_NO_CHANGE;
        case -2:
            return SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP;
        case -1:
            return SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_DOWN;
        case 1:
            return SwitchEvent3Pos::SWITCH_TOGGLE_DOWN_MIDDLE;
        case 2:
            return SwitchEvent3Pos::SWITCH_TOGGLE_UP_MIDDLE;
        default:
            return SwitchEvent3Pos::SWITCH_EVENT_OUT_OF_RANGE_ERROR;
    }
}

/**
 * @brief 根据当前状态和上一次状态返回按键的跳变事件
 * @note 本函数为内部函数, 请勿在外部调用
 */
RemoteControl::KeyEvent RemoteControl::judgeKeyEvent(KeyStatus currentStatus, KeyStatus lastStatus)
{
    switch (static_cast<int8_t>(currentStatus) - static_cast<int8_t>(lastStatus)) {
        case 0:
            return KeyEvent::KEY_NO_CHANGE;
        case -1:
            return KeyEvent::KEY_TOGGLE_PRESS_RELEASE;
        case 1:
            return KeyEvent::KEY_TOGGLE_RELEASE_PRESS;
        default:
            return KeyEvent::KEY_NO_CHANGE;
    }
}

/**
 * @brief 应用摇杆死区
 * @param stickValue 处理前摇杆输入值，范围[-1.0, 1.0]
 * @return fp32 移除死区后的摇杆值，重映射到范围[-1.0, 1.0]
 */
fp32 RemoteControl::applyStickDeadZone(fp32 stickValue)
{
    if (fabs(stickValue) < m_stickDeadZone) {
        return 0.0f;
    }
    if (stickValue > 0.0f) {
        return (stickValue - m_stickDeadZone) / (1.0f - m_stickDeadZone);
    }
    return (stickValue + m_stickDeadZone) / (1.0f - m_stickDeadZone);
}

/******************************************************************************
 *                           Dr16RemoteControl类实现
 ******************************************************************************/

/**
 * @brief 构造函数，初始化遥控器接收数据地址指针和连接状态
 */
Dr16RemoteControl::Dr16RemoteControl(fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_scrollWheel(0.0f),
      m_rightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastRightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_rightSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastLeftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_leftSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_mouseXSpeed(0.0f),
      m_mouseYSpeed(0.0f),
      m_mouseWheelSpeed(0.0f),
      m_mouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseLeftKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseLeftKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_lastMouseRightKeyStatus(KeyStatus::KEY_ERROR),
      m_mouseRightKeyEvent(KeyEvent::KEY_EVENT_NO_UPDATE_ERROR)
{
    // 初始化键盘按键状态数组
    for (uint8_t i = 0; i < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); i++) {
        m_keyboardKeyStatus[i]     = KeyStatus::KEY_ERROR;
        m_lastKeyboardKeyStatus[i] = KeyStatus::KEY_ERROR;
        m_keyboardKeyEvent[i]      = KeyEvent::KEY_EVENT_NO_UPDATE_ERROR;
    }
}

/**
 * @brief 从中断中获取DR16遥控器接收数据地址，更新相关标志位
 * @param data DR16遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */
void Dr16RemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (DR16OriginalUARTRxData *)data;
    m_uartRxTimestamp       = HAL_GetTick(); // 更新接收时间戳
    m_isConnected           = true;          // 接收到数据则认为遥控器连接正常
    m_isDecodeCompleted     = false;         // 标志解码未完成
}

/**
 * @brief 接收DR16遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */
void Dr16RemoteControl::decodeRxData()
{
    // 判断遥控器连接状态，若使用的数据过时超过100ms则认为遥控器断开
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
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

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */
void Dr16RemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;
    m_lastRightSwitchStatus   = m_rightSwitchStatus;
    m_rightSwitchStatus       = (SwitchStatus3Pos)m_originalRxDataPointer->Switch_2;
    m_rightSwitchEvent        = judgeSwitchEvent(m_rightSwitchStatus, m_lastRightSwitchStatus);
    m_lastLeftSwitchStatus    = m_leftSwitchStatus;
    m_leftSwitchStatus        = (SwitchStatus3Pos)m_originalRxDataPointer->Switch_1;
    m_leftSwitchEvent         = judgeSwitchEvent(m_leftSwitchStatus, m_lastLeftSwitchStatus);
    m_lastMouseLeftKeyStatus  = m_mouseLeftKeyStatus;
    m_mouseLeftKeyStatus      = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
    m_mouseLeftKeyEvent       = judgeKeyEvent(m_mouseLeftKeyStatus, m_lastMouseLeftKeyStatus);
    m_lastMouseRightKeyStatus = m_mouseRightKeyStatus;
    m_mouseRightKeyStatus     = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
    m_mouseRightKeyEvent      = judgeKeyEvent(m_mouseRightKeyStatus, m_lastMouseRightKeyStatus);
    for (uint8_t keyIndex = 0; keyIndex < static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER); keyIndex++) {
        m_lastKeyboardKeyStatus[keyIndex] = m_keyboardKeyStatus[keyIndex];
        m_keyboardKeyStatus[keyIndex]     = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> keyIndex & 0x01);
        m_keyboardKeyEvent[keyIndex]      = judgeKeyEvent(m_keyboardKeyStatus[keyIndex], m_lastKeyboardKeyStatus[keyIndex]);
    }
}

/******************************************************************************
 *                          ET08ARemoteControl类实现
 ******************************************************************************/

/**
 * @brief 双参数构造函数，自定义通道配置
 * @param config 遥控器通道配置结构体
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 */
ET08ARemoteControl::ET08ARemoteControl(Config config, fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_protocolData{},
      m_config(config),
      m_switchSA(SwitchStatus2Pos::SWITCH_ERROR),
      m_switchSD(SwitchStatus2Pos::SWITCH_ERROR),
      m_lastSwitchSA(SwitchStatus2Pos::SWITCH_ERROR),
      m_lastSwitchSD(SwitchStatus2Pos::SWITCH_ERROR),
      m_eventSA(SwitchEvent2Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSD(SwitchEvent2Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_switchSB(SwitchStatus3Pos::SWITCH_ERROR),
      m_switchSC(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastSwitchSB(SwitchStatus3Pos::SWITCH_ERROR),
      m_lastSwitchSC(SwitchStatus3Pos::SWITCH_ERROR),
      m_eventSB(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSC(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_knobLD(0.0f),
      m_knobRD(0.0f),
      m_trimmerT1(0.0f),
      m_trimmerT2(0.0f),
      m_trimmerT3(0.0f),
      m_trimmerT4(0.0f),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_rightSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_rightSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SwitchStatus3Pos::SWITCH_ERROR),
      m_leftSwitchEvent(SwitchEvent3Pos::SWITCH_EVENT_NO_UPDATE_ERROR) {}

/**
 * @brief 单参数构造函数，使用默认通道配置
 * @param stickDeadZone 遥控器摇杆死区, 范围:[0.0, 1.0), 默认0.0f
 */
ET08ARemoteControl::ET08ARemoteControl(fp32 stickDeadZone)
    : ET08ARemoteControl(Config{}, stickDeadZone) {}

/**
 * @brief 从中断中获取ET08A遥控器接收数据地址，更新相关标志位
 * @param data ET08A遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */
void ET08ARemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (ET08ARawPacket *)data;
    m_uartRxTimestamp       = HAL_GetTick();
    m_isDecodeCompleted     = false;
}

/**
 * @brief 接收ET08A遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */
void ET08ARemoteControl::decodeRxData()
{
    // 判断遥控器连接状态，若使用的数据过时超过100ms则认为遥控器断开
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer->startByte != 0x0F || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
        return;
    }

    // 遥控器掉线时stopByte为0x0F
    m_isConnected = !(m_originalRxDataPointer->stopByte == 0x0F);

    // 解码遥控器数据, 每次接收数据仅解码一次
    if (m_isDecodeCompleted) return;

    parseET08AProtocol(m_originalRxDataPointer->data, m_protocolData);

    // ET08A遥控器通道值范围是[353, 1694], 中点1024
    // 为保证数值在[-1.0, 1.0]范围内且保证线性性, 取较大的下限偏移量671, 最终归一化数值范围是[-1.0, 0.9985]
    m_rightStickX = (m_protocolData.rightStickX - 1024) / 671.0f;
    m_rightStickY = (m_protocolData.rightStickY - 1024) / 671.0f;
    m_leftStickX  = (m_protocolData.leftStickX - 1024) / 671.0f;
    m_leftStickY  = (m_protocolData.leftStickY - 1024) / 671.0f;
    m_knobLD      = (m_protocolData.knobLD - 1024) / 671.0f;
    m_knobRD      = (m_protocolData.knobRD - 1024) / 671.0f;
    m_trimmerT1   = (m_protocolData.trimmerT1 - 1024) / 671.0f;
    m_trimmerT2   = (m_protocolData.trimmerT2 - 1024) / 671.0f;
    m_trimmerT3   = (m_protocolData.trimmerT3 - 1024) / 671.0f;
    m_trimmerT4   = (m_protocolData.trimmerT4 - 1024) / 671.0f;

    m_isDecodeCompleted = true;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */
void ET08ARemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;
    decodeRxData();

    m_lastSwitchSA = m_switchSA;
    m_lastSwitchSB = m_switchSB;
    if (m_config.switchSASB != ET08AChannelIndex::CH_NONE) { // SA和SB复用通道, SB微调
        if (m_protocolData.switchSASB < 1024) {
            m_switchSA = SwitchStatus2Pos::SWITCH_UP;
            switch ((m_protocolData.switchSASB - 151) / 201) {
                case 0:
                    m_switchSB = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    break;
            }
        } else {
            m_switchSA = SwitchStatus2Pos::SWITCH_DOWN;
            switch ((m_protocolData.switchSASB - 1493) / 201) {
                case 0:
                    m_switchSB = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    break;
            }
        }
    } else { // SA和SB使用独立通道
        if (m_protocolData.switchSA < 1024) {
            m_switchSA = SwitchStatus2Pos::SWITCH_UP;
        } else {
            m_switchSA = SwitchStatus2Pos::SWITCH_DOWN;
        }
        if (m_protocolData.switchSB < 1024) {
            m_switchSB = SwitchStatus3Pos::SWITCH_UP;
        } else if (m_protocolData.switchSB > 1024) {
            m_switchSB = SwitchStatus3Pos::SWITCH_DOWN;
        } else {
            m_switchSB = SwitchStatus3Pos::SWITCH_MIDDLE;
        }
    }
    m_eventSA = judgeSwitchEvent(m_switchSA, m_lastSwitchSA);
    m_eventSB = judgeSwitchEvent(m_switchSB, m_lastSwitchSB);

    m_lastSwitchSC = m_switchSC;
    m_lastSwitchSD = m_switchSD;
    if (m_config.switchSCSD != ET08AChannelIndex::CH_NONE) { // SC和SD复用通道, SC微调
        if (m_protocolData.switchSCSD < 1024) {
            m_switchSD = SwitchStatus2Pos::SWITCH_UP;
            switch ((m_protocolData.switchSCSD - 151) / 201) {
                case 0:
                    m_switchSC = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    break;
            }
        } else {
            m_switchSD = SwitchStatus2Pos::SWITCH_DOWN;
            switch ((m_protocolData.switchSCSD - 1493) / 201) {
                case 0:
                    m_switchSC = SwitchStatus3Pos::SWITCH_UP;
                    break;
                case 1:
                    m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
                    break;
                case 2:
                    m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
                    break;
                default:
                    break;
            }
        }
    } else { // SC和SD使用独立通道
        if (m_protocolData.switchSC < 1024) {
            m_switchSC = SwitchStatus3Pos::SWITCH_UP;
        } else if (m_protocolData.switchSC > 1024) {
            m_switchSC = SwitchStatus3Pos::SWITCH_DOWN;
        } else {
            m_switchSC = SwitchStatus3Pos::SWITCH_MIDDLE;
        }
        if (m_protocolData.switchSD < 1024) {
            m_switchSD = SwitchStatus2Pos::SWITCH_UP;
        } else {
            m_switchSD = SwitchStatus2Pos::SWITCH_DOWN;
        }
    }
    m_eventSC      = judgeSwitchEvent(m_switchSC, m_lastSwitchSC);
    m_eventSD = judgeSwitchEvent(m_switchSD, m_lastSwitchSD);
}

/*
    @brief 解析ET08A遥控器协议数据
    @param buffer 原始协议数据缓冲区指针
*/
void ET08ARemoteControl::parseET08AProtocol(const uint8_t *buffer, ET08AProtocolData &out) const
{
    if (buffer == nullptr) return;

    uint16_t temp_ch[16];

    temp_ch[0]  = ((buffer[0] | buffer[1] << 8) & 0x07FF);
    temp_ch[1]  = ((buffer[1] >> 3 | buffer[2] << 5) & 0x07FF);
    temp_ch[2]  = ((buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF);
    temp_ch[3]  = ((buffer[4] >> 1 | buffer[5] << 7) & 0x07FF);
    temp_ch[4]  = ((buffer[5] >> 4 | buffer[6] << 4) & 0x07FF);
    temp_ch[5]  = ((buffer[6] >> 7 | buffer[7] << 1 | buffer[8] << 9) & 0x07FF);
    temp_ch[6]  = ((buffer[8] >> 2 | buffer[9] << 6) & 0x07FF);
    temp_ch[7]  = ((buffer[9] >> 5 | buffer[10] << 3) & 0x07FF);
    temp_ch[8]  = ((buffer[11] | buffer[12] << 8) & 0x07FF);
    temp_ch[9]  = ((buffer[12] >> 3 | buffer[13] << 5) & 0x07FF);
    temp_ch[10] = ((buffer[13] >> 6 | buffer[14] << 2 | buffer[15] << 10) & 0x07FF);
    temp_ch[11] = ((buffer[15] >> 1 | buffer[16] << 7) & 0x07FF);
    temp_ch[12] = ((buffer[16] >> 4 | buffer[17] << 4) & 0x07FF);
    temp_ch[13] = ((buffer[17] >> 7 | buffer[18] << 1 | buffer[19] << 9) & 0x07FF);
    temp_ch[14] = ((buffer[19] >> 2 | buffer[20] << 6) & 0x07FF);
    temp_ch[15] = ((buffer[20] >> 5 | buffer[21] << 3) & 0x07FF);

    // 根据配置映射通道数据
    auto getChannelValue = [&temp_ch](ET08AChannelIndex index) -> uint16_t {
        return (index == ET08AChannelIndex::CH_NONE) ? 1024 : temp_ch[static_cast<uint8_t>(index)];
    };

    out.rightStickX = getChannelValue(m_config.rightStickJ1X);
    out.rightStickY = getChannelValue(m_config.rightStickJ2Y);
    out.leftStickX  = getChannelValue(m_config.leftStickJ4X);
    out.leftStickY  = getChannelValue(m_config.leftStickJ3Y);

    out.switchSA = getChannelValue(m_config.switchSA);
    out.switchSB = getChannelValue(m_config.switchSB);
    out.switchSC = getChannelValue(m_config.switchSC);
    out.switchSD = getChannelValue(m_config.switchSD);

    out.switchSASB = getChannelValue(m_config.switchSASB);
    out.switchSCSD = getChannelValue(m_config.switchSCSD);

    out.knobLD = getChannelValue(m_config.knobLD);
    out.knobRD = getChannelValue(m_config.knobRD);

    out.trimmerT1 = getChannelValue(m_config.trimmerT1);
    out.trimmerT2 = getChannelValue(m_config.trimmerT2);
    out.trimmerT3 = getChannelValue(m_config.trimmerT3);
    out.trimmerT4 = getChannelValue(m_config.trimmerT4);
}
