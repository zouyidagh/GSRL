/**
 ******************************************************************************
 * @file           : dvc_motor.hpp
 * @brief          : header file for dvc_motor.cpp
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
#include "alg_pid.hpp"
#include "drv_can.h"
#include <cstdint>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 电机基类
 * @note 该类不可实例化
 * @details 包含电机通用的通信、状态、控制器相关变量和方法
 * @details 该类的派生类需实现以下方法：
 * - convertControllerOutputToMotorControlData 将控制器输出转换为电机控制CAN数据
 * - decodeCanRxMessage 从CAN接收到的数据中解析出电机反馈数据并检查电机是否在线
 */
class Motor
{
protected:
    // 通信
    uint32_t m_motorControlMessageID;         // 电机CAN控制消息ID
    uint32_t m_motorFeedbackMessageID;        // 电机CAN反馈消息ID
    CAN_TxHeaderTypeDef m_motorControlHeader; // 电机CAN控制消息头
    uint8_t m_motorControlData[8];            // 电机CAN控制数据
    uint8_t m_motorFeedbackData[8];           // 电机CAN反馈数据
    uint8_t m_motorFeedbackSequence;          // 电机反馈数据序号, 用于中断接收离线判断
    uint8_t m_motorLastFeedbackSequence;      // 上一次反馈数据序号
    // 当前状态
    fp32 m_currentAngle;               // rad [0, 2PI)
    fp32 m_lastAngle;                  // rad [0, 2PI)
    fp32 m_currentAngularVelocity;     // rad/s
    int16_t m_roundCount;              // 整圈数，用于计算小数圈数
    fp32 m_zeroAngle;                  // 零位角度，用于计算小数圈数
    fp32 m_currentRevolutions;         // 小数圈数 n*2pi(rad)
    int16_t m_currentTorqueCurrent;
    int8_t m_temperature;              // ℃
    uint8_t m_motorFeedbackErrorCount;
    bool m_isMotorConnected;
    // 目标状态
    fp32 m_targetAngle;                // rad [0, 2PI)
    fp32 m_targetAngularVelocity;      // rad/s
    fp32 m_targetRevolutions;          // 圈n*2pi(rad)
    int16_t m_targetTorqueCurrent;
    // 控制器
    Controller *m_controller;          // 符合 Controller 接口的控制器
    fp32 m_controllerOutput;
    bool m_controllerOutputPolarity;
    uint16_t m_encoderOffset;

public:
    virtual ~Motor() = default;
    // 通信相关
    uint32_t getMotorControlMessageID();
    uint32_t getMotorFeedbackMessageID();
    const CAN_TxHeaderTypeDef *getMotorControlHeader() const;
    const uint8_t *getMotorControlData();
    virtual bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
    virtual bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
    const uint8_t *getMotorFeedbackData() const;
    // 获取当前状态相关
    fp32 getCurrentAngle() const;
    fp32 getCurrentAngularVelocity() const;
    void resetCurrentRevolutionsToZero();
    fp32 getCurrentRevolutions() const;
    int16_t getCurrentTorqueCurrent() const;
    int8_t getTemperature() const;
    bool isMotorConected() const;
    // 设置目标状态相关
    void setTargetAngle(fp32 targetAngle);
    void setTargetAngularVelocity(fp32 targetAngularVelocity);
    void setTargetRevolutions(fp32 targetRevolutions);
    void setTargetTorqueCurrent(int16_t targetTorqueCurrent);
    // 控制器相关
    void setController(Controller *controller);
    void setControllerOutputPolarity(bool polarity);
    void openloopControl(fp32 controlValue);
    fp32 angleClosedloopControl();
    fp32 angleClosedloopControl(fp32 targetAngle);
    fp32 angularVelocityClosedloopControl();
    fp32 angularVelocityClosedloopControl(fp32 targetAngularVelocity);
    fp32 revolutionsClosedloopControl();
    fp32 revolutionsClosedloopControl(fp32 targetRevolutions);
    int16_t torqueCurrentClosedloopControl();
    int16_t torqueCurrentClosedloopControl(int16_t targetTorqueCurrent);
    fp32 externalClosedloopControl(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize);

protected:
    Motor(uint32_t canControlID, uint32_t canFeedbackID, Controller *controller, uint16_t encoderOffset = 0);
    virtual bool decodeCanRxMessage(const can_rx_message_t &rxMessage) = 0;
    virtual void convertControllerOutputToMotorControlData() = 0;
    fp32 updateCurrentRevolutions();
    inline void increaseMotorFeedbackErrorCount();
    inline void clearMotorFeedbackErrorCount();
};

/**
 * @brief GM6020电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制大疆6020电机
 */
class MotorGM6020 : public Motor
{
protected:
    uint8_t m_djiMotorID;         // 大疆电机ID，6020电机对应1~7
    uint16_t m_encoderHistory[2]; // 0:当前值 1:上一次值
    int16_t m_currentRPMSpeed;    // RPM

public:
    MotorGM6020(uint8_t dji6020MotorID, Controller *controller, uint16_t encoderOffset = 0);
    void convertControllerOutputToMotorControlData() override;
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    uint8_t getDjiMotorID() const;
    MotorGM6020 operator+(const MotorGM6020 &otherMotor) const;
};

/**
 * @brief M3508电机类
 * @details 在GM6020电机类的基础上适配
 */
class MotorM3508 : public MotorGM6020
{
protected:
    uint8_t m_gearboxRatio; // 减速比倒数，用于换算电机转速、角速度，电机角度、圈数不受此值影响，默认为1即不换算

public:
    MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset = 0, uint8_t gearboxRatio = 1);
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
};

/**
 * @brief 达妙DMJ4310电机类
 * @note 请使用达妙电机默认固件，本类基于MIT模式单力矩输出控制帧，其余计算(如PID)在stm32上实现
 * @note 使用前先用达妙电机调试助手查看电机驱动板的PMAX、VMAX、TMAX参数，确保控制板与代码中的参数一致，否则会导致数据解包错误
 * @note 请将PMAX值设置位PI(3.1415926), 请将PMAX值设置位PI(3.141593), 重要！！！
 * @note 建议合理设置电机CANTimeout, 以避免电机掉线不受控, 本类会在电机重连后自动清除错误状态并使能电机
 * @details 该类实现了Motor类的纯虚函数，用于控制达妙4310电机
 */
class MotorDM4310 : public Motor
{
public:
    enum DMMotorState : uint8_t
    {
        DISABLE = 0x00,
        ENABLE = 0x01,
        OVER_VOLTAGE = 0x08,
        UNDER_VOLTAGE = 0x09,
        OVER_CURRENT = 0x0A,
        MOSFET_OVER_TEMPERATURE = 0x0B,
        COLI_OVER_TEMPERATURE = 0x0C,
        COMMUNICATION_LOST = 0x0D,
        OVERLOAD = 0x0E
    };

protected:
    DMMotorState m_motorState;
    fp32 m_dmEncoderPosition;
    fp32 m_mosfetTemperature;
    fp32 m_coliTemperature;
    bool m_setZeroPositionFlag;
    const fp32 PMAX, VMAX, TMAX;

public:
    MotorDM4310(uint8_t dmControlID, uint8_t dmMasterID, fp32 pmax, fp32 vmax, fp32 tmax, Controller *controller);
    void convertControllerOutputToMotorControlData() override;
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void setMotorZeroPosition();
};

/**
 * @brief 云深处 J60 电机类
 * @details 适用于 J60 电机 CAN 通信模式 (1Mbps)
 */
class MotorJ60 : public Motor
{
public:
    uint8_t m_jointID; // 电机关节ID (默认为1)
    enum MotorState
    {
        DISABLED = 0, // 失能
        ENABLED,      // 使能
        ERROR         // 错误保护
    };

    struct J60Error
    {
        bool underVoltage;   // < 18V
        bool overVoltage;    // > 36V
        bool overCurrent;    // > 28A
        bool motorOverTemp;  // > 125C
        bool driverOverTemp; // > 120C
        uint8_t rawErrorCode;
    };

    // DeepJ60 ID Gen: ID = (CmdID << 5) | MotorID
    // 移除之前的 Flag bit 逻辑
     static uint16_t generateCanID(uint8_t jointID, uint8_t cmdID){
        uint16_t temp = (jointID & 0x1F) | ((cmdID & 0x3F) << 5);
        return temp;
    }
    MotorJ60(uint8_t motorID, Controller *controller);
    
    void convertControllerOutputToMotorControlData() override;
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;

    // 状态管理
    void enable();
    void disable();
    void constructRTR(uint8_t cmdID);
    void clearError();
    void setZeroPosition();
    
    // 控制接口
    void setControlParams(fp32 p, fp32 v, fp32 t, fp32 kp, fp32 kd);
    
    // 获取状态
    MotorState getState() const { return m_state; }
    J60Error getError() const { return m_error; }

    // 调试辅助
    uint8_t getEnableCmdTxCountDebug() const { return m_enableCmdTxCount; }

private:
    MotorState m_state;
    J60Error m_error;

    // 电机ID (0-15)
    uint8_t m_motorID;

    // 命令索引常量
    static constexpr uint8_t CMD_DISABLE = 1;
    static constexpr uint8_t CMD_ENABLE = 2;
    static constexpr uint8_t CMD_CONTROL = 4;
    static constexpr uint8_t CMD_CLEAR_ERROR = 17;
    static constexpr uint8_t CMD_GET_STATUS = 23;


    // 当前发送的命令索引
    uint8_t m_sendingCmdIndex;

    // 控制参数缓存
    fp32 m_cmdP, m_cmdV, m_cmdT, m_cmdKp, m_cmdKd;
    uint8_t m_enableCmdTxCount; // 使能命令发送计数
    uint8_t m_zeroCmdTxCount;   // 零点设置命令发送计数

    // 限制常量（按J60协议）
    const fp32 P_MIN = -40.0f, P_MAX = 40.0f;
    const fp32 V_MIN = -40.0f, V_MAX = 40.0f;
    const fp32 T_MIN = -40.0f, T_MAX = 40.0f;
    const fp32 KP_MIN = 0.0f, KP_MAX = 1023.0f;
    const fp32 KD_MIN = 0.0f, KD_MAX = 51.0f; // 协议规定范围

    void setControlHeader(uint8_t cmdID, uint8_t dlc);
   
};


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
