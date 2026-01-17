/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.hpp
 * @brief          : header file for dvc_remotecontrol.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "gsrl_common.h"
#include "drv_uart.h"
#include <cstdint>
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 遥控器基类
 * @note 该类不可实例化
 */
class RemoteControl
{
public:
    // 遥控器双档拨杆状态
    enum class SwitchStatus2Pos : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // 遥控器双档拨杆跳变事件
    enum class SwitchEvent2Pos : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_DOWN,
        SWITCH_TOGGLE_DOWN_UP,
        SWITCH_EVENT_NO_UPDATE_ERROR    = -1, // 检查RemoteControl::updateEvent()函数是否提前调用
        SWITCH_EVENT_STATUS_ERROR       = -2, // 检查当前或上一次拨杆状态是否为SWITCH_ERROR
        SWITCH_EVENT_OUT_OF_RANGE_ERROR = -3  // 检查当前或上一次拨杆状态是否超出枚举值范围(遥控数据解包错误)
    };

    // 遥控器三档拨杆状态
    enum class SwitchStatus3Pos : int8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_MIDDLE,
        SWITCH_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // 遥控器三档拨杆跳变事件
    enum class SwitchEvent3Pos : int8_t {
        SWITCH_NO_CHANGE,
        SWITCH_TOGGLE_UP_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_UP,
        SWITCH_TOGGLE_DOWN_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_DOWN,
        SWITCH_EVENT_NO_UPDATE_ERROR    = -1, // 检查RemoteControl::updateEvent()函数是否提前调用
        SWITCH_EVENT_STATUS_ERROR       = -2, // 检查当前或上一次拨杆状态是否为SWITCH_ERROR
        SWITCH_EVENT_OUT_OF_RANGE_ERROR = -3  // 检查当前或上一次拨杆状态是否超出枚举值范围(遥控数据解包错误)
    };

    // 遥控器按键状态
    enum class KeyStatus : int8_t {
        KEY_RELEASE = 0,
        KEY_PRESS,
        KEY_ERROR = -1 // 检查m_originalRxDataPointer是否为空
    };

    // 遥控器按键跳变事件
    enum class KeyEvent : int8_t {
        KEY_NO_CHANGE,
        KEY_TOGGLE_PRESS_RELEASE,
        KEY_TOGGLE_RELEASE_PRESS,
        KEY_EVENT_NO_UPDATE_ERROR = -1 // 检查Dr16RemoteControl::updateEvent()函数是否提前调用
    };

protected:
    // 遥控器连接状态检测
    uint32_t m_uartRxTimestamp; // 使用毫秒级HAL_GetTick()获取, 判断遥控器连接状态
    bool m_isConnected;
    bool m_isDecodeCompleted; // 解码完成标志
    fp32 m_stickDeadZone;     // 遥控器摇杆死区

public:
    virtual ~RemoteControl()                               = default;
    virtual void receiveRxDataFromISR(const uint8_t *data) = 0;
    virtual void decodeRxData()                            = 0;
    virtual void updateEvent()                             = 0;
    bool isConnected()
    {
        decodeRxData();
        return m_isConnected;
    }

protected:
    RemoteControl(fp32 stickDeadZone = 0.0f);
    SwitchEvent2Pos judgeSwitchEvent(SwitchStatus2Pos currentStatus, SwitchStatus2Pos lastStatus);
    SwitchEvent3Pos judgeSwitchEvent(SwitchStatus3Pos currentStatus, SwitchStatus3Pos lastStatus);
    KeyEvent judgeKeyEvent(KeyStatus currentStatus, KeyStatus lastStatus);
    fp32 applyStickDeadZone(fp32 stickValue);
};

/**
 * @brief 大疆DR16遥控器类，用于解码DR16遥控器接收数据
 * @note 使用前需确保receiveDr16RxDataFromISR方法在对应UART接收中断服务函数中被调用
 */
class Dr16RemoteControl : public RemoteControl
{
public:
    // DR16遥控器原始数据结构体
    struct DR16OriginalUARTRxData {
        uint64_t Channel_0 : 11;
        uint64_t Channel_1 : 11;
        uint64_t Channel_2 : 11;
        uint64_t Channel_3 : 11;
        uint64_t Switch_2 : 2;
        uint64_t Switch_1 : 2;
        int16_t Mouse_X;
        int16_t Mouse_Y;
        int16_t Mouse_Z;
        uint64_t Mouse_Left_Key : 8;
        uint64_t Mouse_Right_Key : 8;
        uint64_t Keyboard_Key : 16;
        uint64_t Channel_4 : 11;
    } __attribute__((packed));

    // DR16遥控器键盘按键对应索引
    enum class KeyboardKeyIndex : uint8_t {
        KEY_W = 0,
        KEY_S,
        KEY_A,
        KEY_D,
        KEY_SHIFT,
        KEY_CTRL,
        KEY_Q,
        KEY_E,
        KEY_R,
        KEY_F,
        KEY_G,
        KEY_Z,
        KEY_X,
        KEY_C,
        KEY_V,
        KEY_B,
        KEY_TOTAL_NUMBER // 键盘按键枚举值总数
    };

private:
    DR16OriginalUARTRxData *m_originalRxDataPointer; // DR16遥控器原始接收数据指针
    // DR16遥控器解码数据
    fp32 m_rightStickX;
    fp32 m_rightStickY;
    fp32 m_leftStickX;
    fp32 m_leftStickY;
    fp32 m_scrollWheel;
    SwitchStatus3Pos m_rightSwitchStatus;
    SwitchStatus3Pos m_lastRightSwitchStatus; // 上一次右拨杆状态
    SwitchEvent3Pos m_rightSwitchEvent;
    SwitchStatus3Pos m_leftSwitchStatus;
    SwitchStatus3Pos m_lastLeftSwitchStatus; // 上一次左拨杆状态
    SwitchEvent3Pos m_leftSwitchEvent;
    fp32 m_mouseXSpeed;
    fp32 m_mouseYSpeed;
    fp32 m_mouseWheelSpeed;
    KeyStatus m_mouseLeftKeyStatus;
    KeyStatus m_lastMouseLeftKeyStatus; // 上一次鼠标左键状态
    KeyEvent m_mouseLeftKeyEvent;
    KeyStatus m_mouseRightKeyStatus;
    KeyStatus m_lastMouseRightKeyStatus; // 上一次鼠标右键状态
    KeyEvent m_mouseRightKeyEvent;
    KeyStatus m_keyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];
    KeyStatus m_lastKeyboardKeyStatus[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)]; // 上一次键盘按键状态
    KeyEvent m_keyboardKeyEvent[static_cast<uint8_t>(KeyboardKeyIndex::KEY_TOTAL_NUMBER)];

public:
    Dr16RemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data) override;
    void decodeRxData() override;
    void updateEvent() override;

    fp32 getRightStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickX);
    }
    fp32 getRightStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickY);
    }
    fp32 getLeftStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickX);
    }
    fp32 getLeftStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickY);
    }
    SwitchStatus3Pos getRightSwitchStatus()
    {
        if (m_originalRxDataPointer == nullptr) return SwitchStatus3Pos::SWITCH_ERROR;
        return m_rightSwitchStatus = (SwitchStatus3Pos)m_originalRxDataPointer->Switch_2;
    }
    SwitchEvent3Pos getRightSwitchEvent()
    {
        return m_rightSwitchEvent;
    }
    SwitchStatus3Pos getLeftSwitchStatus()
    {
        if (m_originalRxDataPointer == nullptr) return SwitchStatus3Pos::SWITCH_ERROR;
        return m_leftSwitchStatus = (SwitchStatus3Pos)m_originalRxDataPointer->Switch_1;
    }
    SwitchEvent3Pos getLeftSwitchEvent()
    {
        return m_leftSwitchEvent;
    }
    fp32 getMouseX()
    {
        decodeRxData();
        return m_mouseXSpeed;
    }
    fp32 getMouseY()
    {
        decodeRxData();
        return m_mouseYSpeed;
    }
    fp32 getMouseWheel()
    {
        decodeRxData();
        return m_mouseWheelSpeed;
    }
    KeyStatus getMouseLeftKeyStatus()
    {
        if (m_originalRxDataPointer == nullptr) return KeyStatus::KEY_ERROR;
        return m_mouseLeftKeyStatus = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
    }
    KeyEvent getMouseLeftKeyEvent()
    {
        return m_mouseLeftKeyEvent;
    }
    KeyStatus getMouseRightKeyStatus()
    {
        if (m_originalRxDataPointer == nullptr) return KeyStatus::KEY_ERROR;
        return m_mouseRightKeyStatus = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
    }
    KeyEvent getMouseRightKeyEvent()
    {
        return m_mouseRightKeyEvent;
    }
    KeyStatus getKeyboardKeyStatus(KeyboardKeyIndex keyIndex)
    {
        if (m_originalRxDataPointer == nullptr) return KeyStatus::KEY_ERROR;
        return m_keyboardKeyStatus[static_cast<uint8_t>(keyIndex)] = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> static_cast<uint8_t>(keyIndex) & 0x01);
    }
    KeyEvent getKeyboardKeyEvent(KeyboardKeyIndex keyIndex)
    {
        return m_keyboardKeyEvent[static_cast<uint8_t>(keyIndex)];
    }
};

/**
 * @brief 天地飞ET08A遥控器类 (W.BUS)
 * @note 详细使用文档见Documentation/Device/ET08ARemoteControl.md
 */
class ET08ARemoteControl : public RemoteControl
{
public:
    // ET08A遥控器原始数据结构体
    struct ET08ARawPacket {
        uint8_t startByte;
        uint8_t data[22];
        uint8_t stopByte;
    } __attribute__((packed));

    // ET08A通道索引枚举
    enum class ET08AChannelIndex : uint8_t {
        CH_1 = 0,
        CH_2,
        CH_3,
        CH_4,
        CH_5,
        CH_6,
        CH_7,
        CH_8,
        CH_NONE = 0xFF
    };

    // ET08A配置结构体
    struct Config {
        ET08AChannelIndex rightStickJ1X = ET08AChannelIndex::CH_1;
        ET08AChannelIndex rightStickJ2Y = ET08AChannelIndex::CH_3;
        ET08AChannelIndex leftStickJ3Y  = ET08AChannelIndex::CH_2;
        ET08AChannelIndex leftStickJ4X  = ET08AChannelIndex::CH_4;
        ET08AChannelIndex switchSA      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSB      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSC      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSD      = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex switchSASB    = ET08AChannelIndex::CH_5; // 复用SA和SB通道,SB微调
        ET08AChannelIndex switchSCSD    = ET08AChannelIndex::CH_6; // 复用SC和SD通道,SC微调
        ET08AChannelIndex knobLD        = ET08AChannelIndex::CH_7;
        ET08AChannelIndex knobRD        = ET08AChannelIndex::CH_8;
        ET08AChannelIndex trimmerT1     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT2     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT3     = ET08AChannelIndex::CH_NONE;
        ET08AChannelIndex trimmerT4     = ET08AChannelIndex::CH_NONE;
    };

private:
    struct ET08AProtocolData {
        uint16_t rightStickX;
        uint16_t rightStickY;
        uint16_t leftStickX;
        uint16_t leftStickY;
        uint16_t switchSA;
        uint16_t switchSB;
        uint16_t switchSC;
        uint16_t switchSD;
        uint16_t switchSASB;
        uint16_t switchSCSD;
        uint16_t knobLD;
        uint16_t knobRD;
        uint16_t trimmerT1;
        uint16_t trimmerT2;
        uint16_t trimmerT3;
        uint16_t trimmerT4;
    };

    ET08ARawPacket *m_originalRxDataPointer;
    ET08AProtocolData m_protocolData; // 解码后的协议数据缓存
    Config m_config;                  // 通道配置结构体

    // ET08A 特有控件数据
    SwitchStatus2Pos m_switchSA, m_switchSD; // SA和SD只有UP和DOWN两个挡位
    SwitchStatus2Pos m_lastSwitchSA, m_lastSwitchSD;
    SwitchEvent2Pos m_eventSA, m_eventSD;
    SwitchStatus3Pos m_switchSB, m_switchSC;
    SwitchStatus3Pos m_lastSwitchSB, m_lastSwitchSC;
    SwitchEvent3Pos m_eventSB, m_eventSC;
    fp32 m_knobLD, m_knobRD;
    fp32 m_trimmerT1, m_trimmerT2, m_trimmerT3, m_trimmerT4;

    // 标准遥控器数据
    fp32 m_rightStickX, m_rightStickY;
    fp32 m_leftStickX, m_leftStickY;
    SwitchStatus3Pos m_rightSwitchStatus;
    SwitchEvent3Pos m_rightSwitchEvent;
    SwitchStatus3Pos m_leftSwitchStatus;
    SwitchEvent3Pos m_leftSwitchEvent;

public:
    ET08ARemoteControl(Config config, fp32 stickDeadZone = 0.0f);
    ET08ARemoteControl(fp32 stickDeadZone = 0.0f);

    void receiveRxDataFromISR(const uint8_t *data) override;
    void decodeRxData() override;
    void updateEvent() override;
    void parseET08AProtocol(const uint8_t *buffer, ET08AProtocolData &out) const;

    void setConfig(const Config &config)
    {
        m_config            = config;
        m_isDecodeCompleted = false;
    }
    const Config &getConfig() const
    {
        return m_config;
    }
    SwitchStatus2Pos getSwitchSA()
    {
        return m_switchSA;
    }
    SwitchStatus3Pos getSwitchSB()
    {
        return m_switchSB;
    }
    SwitchStatus3Pos getSwitchSC()
    {
        return m_switchSC;
    }
    SwitchStatus2Pos getSwitchSD()
    {
        return m_switchSD;
    }
    SwitchEvent2Pos getSwitchEventSA()
    {
        return m_eventSA;
    }
    SwitchEvent3Pos getSwitchEventSB()
    {
        return m_eventSB;
    }
    SwitchEvent3Pos getSwitchEventSC()
    {
        return m_eventSC;
    }
    SwitchEvent2Pos getSwitchEventSD()
    {
        return m_eventSD;
    }
    fp32 getKnobLD()
    {
        decodeRxData();
        return m_knobLD;
    }
    fp32 getKnobRD()
    {
        decodeRxData();
        return m_knobRD;
    }
    fp32 getTrimmerT1()
    {
        decodeRxData();
        return m_trimmerT1;
    }
    fp32 getTrimmerT2()
    {
        decodeRxData();
        return m_trimmerT2;
    }
    fp32 getTrimmerT3()
    {
        decodeRxData();
        return m_trimmerT3;
    }
    fp32 getTrimmerT4()
    {
        decodeRxData();
        return m_trimmerT4;
    }
    // 标准获取函数
    fp32 getRightStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickX);
    }
    fp32 getRightStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_rightStickY);
    }
    fp32 getLeftStickX()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickX);
    }
    fp32 getLeftStickY()
    {
        decodeRxData();
        return applyStickDeadZone(m_leftStickY);
    }
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
