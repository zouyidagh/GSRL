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

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 大疆DR16遥控器类，用于解码DR16遥控器接收数据
 * @note 使用前需确保receiveDr16RxDataFromISR方法在对应UART接收中断服务函数中被调用
 */
class Dr16RemoteControl
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

    // DR16遥控器拨杆状态
    enum SwitchStatus : uint8_t {
        SWITCH_UP = 1,
        SWITCH_DOWN,
        SWITCH_MIDDLE,
        SWITCH_TOGGLE_UP_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_UP,
        SWITCH_TOGGLE_DOWN_MIDDLE,
        SWITCH_TOGGLE_MIDDLE_DOWN
    };

    // DR16遥控器按键状态
    enum KeyStatus : uint8_t {
        KEY_RELEASE = 0,
        KEY_PRESS,
        KEY_TOGGLE_PRESS_RELEASE,
        KEY_TOGGLE_RELEASE_PRESS
    };

    // DR16遥控器键盘按键对应索引
    enum KeyboardKeyIndex : uint8_t {
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
        KEY_B
    };

private:
    DR16OriginalUARTRxData *m_originalRxDataPointer; // DR16遥控器原始接收数据指针

    // DR16遥控器解码数据
    fp32 m_rightStickX;
    fp32 m_rightStickY;
    fp32 m_leftStickX;
    fp32 m_leftStickY;
    fp32 m_scrollWheel;
    SwitchStatus m_rightSwitch;
    SwitchStatus m_lastRightSwitch; // 上一次右拨杆状态
    SwitchStatus m_leftSwitch;
    SwitchStatus m_lastLeftSwitch; // 上一次左拨杆状态
    fp32 m_mouseXSpeed;
    fp32 m_mouseYSpeed;
    fp32 m_mouseWheelSpeed;
    KeyStatus m_mouseLeftKey;
    KeyStatus m_lastMouseLeftKey; // 上一次鼠标左键状态
    KeyStatus m_mouseRightKey;
    KeyStatus m_lastMouseRightKey; // 上一次鼠标右键状态
    KeyStatus m_keyboardKey[16];
    KeyStatus m_lastKeyboardKey[16]; // 上一次键盘按键状态

    // 遥控器连接状态检测
    uint32_t m_uartRxTimestamp; // 使用毫秒级HAL_GetTick()获取, 判断遥控器连接状态
    bool m_isDr16Connected;

    bool m_isDecodeCompleted; // 解码完成标志

public:
    Dr16RemoteControl();

    void receiveDr16RxDataFromISR(const uint8_t *data);
    void decodeDr16RxData();

    fp32 getRightStickX()
    {
        decodeDr16RxData();
        return m_rightStickX;
    }
    fp32 getRightStickY()
    {
        decodeDr16RxData();
        return m_rightStickY;
    }
    fp32 getLeftStickX()
    {
        decodeDr16RxData();
        return m_leftStickX;
    }
    fp32 getLeftStickY()
    {
        decodeDr16RxData();
        return m_leftStickY;
    }
    SwitchStatus getRightSwitch()
    {
        m_lastRightSwitch   = m_rightSwitch;
        m_rightSwitch       = (SwitchStatus)m_originalRxDataPointer->Switch_2;
        return judgeSwitchStatus(m_rightSwitch, m_lastRightSwitch);
    }
    SwitchStatus getLeftSwitch()
    {
        m_lastLeftSwitch    = m_leftSwitch;
        m_leftSwitch        = (SwitchStatus)m_originalRxDataPointer->Switch_1;
        return judgeSwitchStatus(m_leftSwitch, m_lastLeftSwitch);
    }
    fp32 getMouseX()
    {
        decodeDr16RxData();
        return m_mouseXSpeed;
    }
    fp32 getMouseY()
    {
        decodeDr16RxData();
        return m_mouseYSpeed;
    }
    fp32 getMouseWheel()
    {
        decodeDr16RxData();
        return m_mouseWheelSpeed;
    }
    KeyStatus getMouseLeftKey()
    {
        m_lastMouseLeftKey  = m_mouseLeftKey;
        m_mouseLeftKey      = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
        return judgeKeyStatus(m_mouseLeftKey, m_lastMouseLeftKey);
    }
    KeyStatus getMouseRightKey()
    {
        m_lastMouseRightKey = m_mouseRightKey;
        m_mouseRightKey     = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
        return judgeKeyStatus(m_mouseRightKey, m_lastMouseRightKey);
    }
    KeyStatus getKeyboardKey(KeyboardKeyIndex keyIndex)
    {
        m_lastKeyboardKey[keyIndex] = m_keyboardKey[keyIndex];
        m_keyboardKey[keyIndex]     = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> keyIndex & 0x01);
        return judgeKeyStatus(m_keyboardKey[keyIndex], m_lastKeyboardKey[keyIndex]);
    }
    bool isDr16RemoteControlConnected()
    {
        decodeDr16RxData();
        return m_isDr16Connected;
    }

private:
    /**
     * @brief 根据当前状态和上一次状态返回拨杆包含跳变的状态
     * @note 本函数为内部函数, 请勿在外部调用
     */
    SwitchStatus judgeSwitchStatus(SwitchStatus currentStatus, SwitchStatus lastStatus)
    {
        switch (currentStatus - lastStatus) {
            case 0:
                return currentStatus;
            case -2:
                return SWITCH_TOGGLE_MIDDLE_UP;
            case -1:
                return SWITCH_TOGGLE_MIDDLE_DOWN;
            case 1:
                return SWITCH_TOGGLE_DOWN_MIDDLE;
            case 2:
                return SWITCH_TOGGLE_UP_MIDDLE;
            default:
                return currentStatus;
        }
    }

    /**
     * @brief 根据当前状态和上一次状态返回按键包含跳变的状态
     * @note 本函数为内部函数, 请勿在外部调用
     */
    KeyStatus judgeKeyStatus(KeyStatus currentStatus, KeyStatus lastStatus)
    {
        switch (currentStatus - lastStatus) {
            case 0:
                return currentStatus;
            case -1:
                return KEY_TOGGLE_PRESS_RELEASE;
            case 1:
                return KEY_TOGGLE_RELEASE_PRESS;
            default:
                return currentStatus;
        }
    }
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
