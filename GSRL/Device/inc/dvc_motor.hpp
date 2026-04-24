/**
 ******************************************************************************
 * @file           : dvc_motor.hpp
 * @brief          : header file for dvc_motor.cpp
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
#include "alg_pid.hpp"
#include "drv_can.h"

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
    fp32 m_currentAngle;           // rad [0, 2PI)
    fp32 m_lastAngle;              // rad [0, 2PI)
    fp32 m_currentAngularVelocity; // rad/s
    int16_t m_roundCount;          // 整圈数，用于计算小数圈数
    fp32 m_zeroAngle;              // 零位角度，用于计算小数圈数
    fp32 m_currentRevolutions;     // 小数圈数 n*2pi(rad)
    int16_t m_currentTorqueCurrent;
    int8_t m_temperature; // ℃
    uint8_t m_motorFeedbackErrorCount;
    bool m_isMotorConnected;
    // 目标状态
    fp32 m_targetAngle;           // rad [0, 2PI)
    fp32 m_targetAngularVelocity; // rad/s
    fp32 m_targetRevolutions;     // 圈n*2pi(rad)
    int16_t m_targetTorqueCurrent;
    // 控制器
    Controller *m_controller; // 符合 Controller 接口的控制器
    fp32 m_controllerOutput;
    bool m_controllerOutputPolarity;
    uint16_t m_encoderOffset;

public:
    virtual ~Motor() = default;
    // 通信相关
    uint32_t getMotorControlMessageID() const;  // 发给电机的控制报文 ID
    uint32_t getMotorFeedbackMessageID() const; // 电机回复的反馈报文 ID
    const CAN_TxHeaderTypeDef *getMotorControlHeader() const;
    const uint8_t *getMotorControlData();
    bool decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size);
    bool decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage);
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
    virtual void convertControllerOutputToMotorControlData()           = 0;
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
    uint8_t m_mergedData[8];      // getMergedControlData 输出缓冲区

public:
    MotorGM6020(uint8_t dji6020MotorID, Controller *controller, uint16_t encoderOffset = 0);
    uint8_t getDjiMotorID() const;
    const uint8_t *getMergedControlData(MotorGM6020 &otherMotor);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief M3508电机类
 * @details 在GM6020电机类的基础上适配
 */
class MotorM3508 : public MotorGM6020
{
protected:
    fp32 m_gearboxRatio; // 减速比倒数，用于换算电机转速、角速度，电机角度、圈数不受此值影响，默认为1即不换算

public:
    MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
};

/**
 * @brief M2006电机类型别名
 * @note 复用M3508的CAN通信协议和控制方法
 */
using MotorM2006 = MotorM3508;

/**
 * @brief 达妙DMJ4310电机类
 * @note 请使用达妙电机默认固件，本类基于MIT模式单力矩输出控制帧，其余计算(如PID)在stm32上实现
 * @note 使用前先用达妙电机调试助手查看电机驱动板的PMAX、VMAX、TMAX参数，确保控制板与构造函数中的参数一致，否则会导致数据解包错误
 * @note 请将PMAX值设置位PI(3.1415926), 请将PMAX值设置位PI(3.141593), 重要！！！
 * @note 建议合理设置电机CANTimeout, 以避免电机掉线不受控, 本类会在电机重连后自动清除错误状态并使能电机
 * @details 该类实现了Motor类的纯虚函数，用于控制达妙4310电机
 */
class MotorDM4310 : public Motor
{
public:
    enum DMMotorState : uint8_t {
        DISABLE                 = 0x00,
        ENABLE                  = 0x01,
        OVER_VOLTAGE            = 0x08,
        UNDER_VOLTAGE           = 0x09,
        OVER_CURRENT            = 0x0A,
        MOSFET_OVER_TEMPERATURE = 0x0B,
        COLI_OVER_TEMPERATURE   = 0x0C,
        COMMUNICATION_LOST      = 0x0D,
        OVERLOAD                = 0x0E
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
    void setMotorZeroPosition();

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief 达妙DM-S2325-1EC电机类型别名
 * @note 复用DMJ4310的MIT控制协议
 * @note 使用前先用达妙电机调试助手查看电机驱动板的PMAX、VMAX、TMAX参数，确保控制板与构造函数中的参数一致，否则会导致数据解包错误
 * @note 请将PMAX值设置位PI(3.1415926), 请将PMAX值设置位PI(3.141593), 重要！！！
 * @note 建议合理设置电机CANTimeout, 以避免电机掉线不受控, 本类会在电机重连后自动清除错误状态并使能电机
 */
using MotorDM2325 = MotorDM4310;

/**
 * @brief 达妙一控四固件电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制达妙"一控四"固件下的电机
 * @details 与大疆GM6020的一控多模式类似，同一条控制帧上可承载4个电机的控制电流
 * @note 控制帧: 电机ID 1~4 -> 0x3FE, 电机ID 5~8 -> 0x4FE, 数据段为小端字节序
 * @note 反馈帧: 反馈ID = 0x300 + 电机ID, 数据段为大端字节序
 * @note 控制电流为标幺值，含义参见达妙力位混控模式中的 i_des 说明
 */
class MotorDMmulti : public Motor
{
protected:
    uint8_t m_dmMotorID;          // 达妙电机ID [1,8]
    uint16_t m_encoderHistory[2]; // 0:当前值 1:上一次值
    int16_t m_currentRPMSpeed;    // 电机速度, 单位RPM(已除以100后的真实值)
    uint8_t m_errorState;         // 反馈帧D[7]错误状态字节, 具体含义详见达妙错误状态说明书
    uint8_t m_mergedData[8];      // getMergedControlData 输出缓冲区

public:
    MotorDMmulti(uint8_t dmMotorID, Controller *controller, uint16_t encoderOffset = 0);
    uint8_t getDmMotorID() const;
    uint8_t getErrorState() const;
    const uint8_t *getMergedControlData(MotorDMmulti &otherMotor);

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief 瓴控MG系列电机类
 * @details 该类实现了Motor类的纯虚函数，用于控制瓴控MG系列电机
 */
class MotorLKMG : public Motor
{
protected:
    uint8_t m_lkMotorID;
    static constexpr uint16_t m_encoderResolution = 65535;
    static constexpr int16_t m_openloopLimit      = 2048;
    uint16_t m_encoderRaw;
    fp32 m_gearboxRatio;     // 减速比
    fp32 m_maxVelocity;      // 最大速度，单位rad/s
    bool m_isMotorClockwise; // 是否顺时针旋转
    bool m_isBraked;         // 是否刹车

public:
    MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset = 0, fp32 gearboxRatio = 1.0f);
    void hardwareAngularVelocityClosedloopControl();
    void hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity);
    void hardwareAngleClosedloopControl();
    void hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise);
    void hardwareRevolutionsClosedloopControl();
    void hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity);
    void setBrake(bool isBraked);
    uint8_t getMotorID() const { return m_lkMotorID; }

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;
};

/**
 * @brief 云深处 DeepRobotics J60 关节电机类
 * @note CAN 波特率 1Mbps，标准帧。使用前请用 J60 调试工具标定零位并设置关节 ID (默认为 1)。
 * @note 构造后，首次 convertControllerOutputToMotorControlData() 会发送 ENABLE 命令，
 *       收到驱动器应答后进入控制模式；基类检测到掉线后会自动重发 ENABLE 重连。
 * @note 默认"板外 PID"模式 (Kp=Kd=0)，扭矩目标由 m_controllerOutput 提供 (单位 Nm，
 *       需与 GSRLMath 框架里的 PID 配合)，可直接使用基类 torqueCurrentClosedloopControl、
 *       externalClosedloopControl、angleClosedloopControl、revolutionsClosedloopControl 等接口。
 * @note 若需"板载 PID"模式 (关节驱动板内部执行 PD)，调用 hardwareMitControl(p, v, t, kp, kd)；
 *       切回板外 PID 请调用 exitHardwareMitMode()。
 * @details 该类实现了 Motor 类的纯虚函数，用于控制云深处 J60 关节电机。
 */
class MotorDeepJ60 : public Motor
{
public:
    enum J60State : uint8_t {
        J60_DISABLED = 0, // 未使能 (上电初始态或 disable 响应已收到)
        J60_ENABLED  = 1, // 使能完成，允许发送控制帧
    };

    enum J60Intent : uint8_t {
        INTENT_ENABLE  = 0, // 默认意图：保持使能
        INTENT_DISABLE = 1, // 用户调用 disable() 后进入
    };

protected:
    uint8_t m_jointID;           // 关节 ID (1~15)
    J60State m_state;            // 实际硬件状态 (由反馈帧更新)
    J60Intent m_intent;          // 用户意图
    fp32 m_currentTorqueNm;      // 反馈扭矩 (Nm)
    int16_t m_mosfetTemperature; // MOSFET 温度 (℃)，协议范围 [-20, 200]
    int16_t m_motorTemperature;  // 电机温度 (℃)，协议范围 [-20, 200]
    bool m_pendingErrorReset;    // clearError() 设置，convert 中消费
    // 硬件 MIT 模式缓存 (hardwareMitControl 使用)
    bool m_useHardwareMitMode;
    fp32 m_mitTargetPosition; // rad, [-40, 40]
    fp32 m_mitTargetVelocity; // rad/s, [-40, 40]
    fp32 m_mitTargetTorque;   // Nm, [-40, 40]
    fp32 m_mitKp;             // [0, 1023]
    fp32 m_mitKd;             // [0, 51]

    // 协议物理量范围
    static constexpr fp32 J60_POSITION_MAX = 40.0f;
    static constexpr fp32 J60_POSITION_MIN = -40.0f;
    static constexpr fp32 J60_VELOCITY_MAX = 40.0f;
    static constexpr fp32 J60_VELOCITY_MIN = -40.0f;
    static constexpr fp32 J60_TORQUE_MAX   = 40.0f;
    static constexpr fp32 J60_TORQUE_MIN   = -40.0f;
    static constexpr fp32 J60_KP_MAX       = 1023.0f;
    static constexpr fp32 J60_KD_MAX       = 51.0f;
    static constexpr fp32 J60_TEMP_MAX     = 200.0f;
    static constexpr fp32 J60_TEMP_MIN     = -20.0f;
    // J60 命令索引
    static constexpr uint8_t CMD_DISABLE     = 1;
    static constexpr uint8_t CMD_ENABLE      = 2;
    static constexpr uint8_t CMD_CONTROL     = 4;
    static constexpr uint8_t CMD_ERROR_RESET = 17;

public:
    MotorDeepJ60(uint8_t jointID, Controller *controller);

    // 用户意图控制
    void enable();     // 意图置为使能 (下一帧尝试重连/重使能)
    void disable();    // 意图置为失能 (下一帧开始发 DISABLE)
    void clearError(); // 发送 ERROR_RESET，后续随重新 ENABLE 恢复

    // 硬件 MIT 模式 (可选，绕开基类 Controller，直接下发五元组)
    void hardwareMitControl(fp32 targetPositionRad, fp32 targetVelocityRadps,
                            fp32 targetTorqueNm, fp32 kp, fp32 kd);
    void exitHardwareMitMode();

    // 状态查询
    J60State getJ60State() const { return m_state; }
    fp32 getCurrentTorqueNm() const { return m_currentTorqueNm; }
    int16_t getMosfetTemperature() const { return m_mosfetTemperature; }
    int16_t getMotorTemperature() const { return m_motorTemperature; }
    uint8_t getJointID() const { return m_jointID; }

protected:
    bool decodeCanRxMessage(const can_rx_message_t &rxMessage) override;
    void convertControllerOutputToMotorControlData() override;

    // 构造 CAN ID: (cmdIdx << 5) | jointField
    static constexpr uint16_t makeCanId(uint8_t jointField, uint8_t cmdIdx)
    {
        return (uint16_t)(((uint16_t)cmdIdx << 5) | (jointField & 0x1Fu));
    }
    void buildEnableFrame();
    void buildDisableFrame();
    void buildErrorResetFrame();
    void buildControlFrame(fp32 pRad, fp32 vRadps, fp32 tNm, fp32 kp, fp32 kd);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
