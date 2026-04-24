/**
 ******************************************************************************
 * @file           : dvc_motor.cpp
 * @brief          : 电机模块
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "alg_general.hpp"
#include "std_typedef.h"
#include <math.h>

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                           Motor电机基类实现
 ******************************************************************************/

/**
 * @brief 获取电机CAN控制消息ID
 * @return uint32_t 电机CAN控制消息ID
 */
uint32_t Motor::getMotorControlMessageID() const
{
    return m_motorControlMessageID;
}

/**
 * @brief 获取电机CAN反馈消息ID
 * @return uint32_t 电机CAN反馈消息ID
 */
uint32_t Motor::getMotorFeedbackMessageID() const
{
    return m_motorFeedbackMessageID;
}

/**
 * @brief 获取电机CAN控制消息头结构体指针
 * @return const CAN_TxHeaderTypeDef* 电机CAN控制消息头指针
 */
const CAN_TxHeaderTypeDef *Motor::getMotorControlHeader() const
{
    return &m_motorControlHeader;
}

/**
 * @brief 获取电机电机CAN控制数组数据, 同时检测电机连接状态
 * @return const uint8_t* 电机CAN控制数据motroControlData[8]
 */
const uint8_t *Motor::getMotorControlData()
{
    // 检测电机连接状态
    if (m_motorFeedbackSequence == m_motorLastFeedbackSequence) {
        increaseMotorFeedbackErrorCount();
    } else {
        m_motorLastFeedbackSequence = m_motorFeedbackSequence; // 滚动更新反馈数据序号
    }

    return m_motorControlData;
}

/**
 * @brief 从CAN接收缓冲数组中从后往前解析电机反馈数据
 * @param rxMessage CAN接收缓冲数组
 * @param Size CAN接收缓冲数组大小
 * @return bool 解析成功返回true，否则返回false
 * @note 从后往前解析, 缓冲数组中数据应从旧到新排列
 * @note Size可小于等于缓冲数组大小，推荐设置为接收到的CAN包数量
 * @example 若FreeRTOS消息队列长度为16:
 * uint8_t rxCount = 0;
 * for (rxCount = 0; rxCount < 16; rxCount++) {
 *     if (osMessageQueueGet(canRxQueueHandle, &rxMessage[rxCount], NULL, 0) != osOK) {
 *         break;
 *     }
 * }
 * motor1.decodeCanRxMessageFromQueue(rxMessage, rxCount);
 * motor2.decodeCanRxMessageFromQueue(rxMessage, rxCount);
 * ...
 */
bool Motor::decodeCanRxMessageFromQueue(const can_rx_message_t *rxMessage, uint8_t Size)
{
    for (uint8_t i = Size; i > 0; i--) {
        if (decodeCanRxMessage(rxMessage[i - 1])) {
            clearMotorFeedbackErrorCount();
            updateCurrentRevolutions();
            return true;
        }
    }
    increaseMotorFeedbackErrorCount(); // 未解析到数据增加错误计数
    return false;
}

/**
 * @brief 从中断服务函数中解析电机反馈数据
 * @param rxMessage CAN接收消息
 * @return bool 解析成功返回true，否则返回false
 * @note 该函数仅供中断服务函数调用
 * @note 返回false可能是ID不匹配，属正常情况
 */
bool Motor::decodeCanRxMessageFromISR(const can_rx_message_t *rxMessage)
{
    if (decodeCanRxMessage(*rxMessage)) {
        m_motorFeedbackSequence++;
        clearMotorFeedbackErrorCount();
        updateCurrentRevolutions();
        return true;
    }
    return false;
}

/**
 * @brief 获取电机CAN反馈原始数据
 * @return const uint8_t* 电机CAN反馈原始数据motorFeedbackData[8]
 */
const uint8_t *Motor::getMotorFeedbackData() const
{
    return m_motorFeedbackData;
}

/**
 * @brief 获取电机当前角度
 * @return fp32 电机当前角度 单位rad [0, 2PI)
 */
fp32 Motor::getCurrentAngle() const
{
    return m_currentAngle;
}

/**
 * @brief 获取电机当前角速度
 * @return fp32 电机当前角速度 单位rad/s
 */
fp32 Motor::getCurrentAngularVelocity() const
{
    return m_currentAngularVelocity;
}

/**
 * @brief 电机旋转圈数清零
 */
void Motor::resetCurrentRevolutionsToZero()
{
    m_currentRevolutions = 0.0f;
    m_roundCount         = 0;
    m_zeroAngle          = m_currentAngle;
    m_lastAngle          = m_currentAngle;
}

/**
 * @brief 获取电机当前旋转圈数
 * @return fp32 电机当前旋转圈数
 */
fp32 Motor::getCurrentRevolutions() const
{
    return m_currentRevolutions;
}

/**
 * @brief 获取电机当前力矩电流
 * @return int16_t 电机当前力矩电流
 */
int16_t Motor::getCurrentTorqueCurrent() const
{
    return m_currentTorqueCurrent;
}

/**
 * @brief 获取电机当前温度
 * @return uint8_t 电机当前温度 单位℃
 */
int8_t Motor::getTemperature() const
{
    return m_temperature;
}

/**
 * @brief 检查电机是否连接
 * @return true 电机连接正常
 * @return false 电机连接异常
 */
bool Motor::isMotorConected() const
{
    return m_isMotorConnected;
}

/**
 * @brief 设置电机目标角度
 * @param targetAngle 电机目标角度 单位rad [0, 2PI)
 */
void Motor::setTargetAngle(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
}

/**
 * @brief 设置电机目标角速度
 * @param targetAngularVelocity 电机目标角速度 单位rad/s
 */
void Motor::setTargetAngularVelocity(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
}

/**
 * @brief 设置电机目标旋转圈数
 * @param targetRevolutions 电机目标旋转圈数
 */
void Motor::setTargetRevolutions(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
}

/**
 * @brief 设置电机目标力矩电流
 * @param targetTorqueCurrent 电机目标力矩电流
 */
void Motor::setTargetTorqueCurrent(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
}

/**
 * @brief 设置电机绑定的控制器
 * @param controller 电机控制器
 */
void Motor::setController(Controller *controller)
{
    m_controller = controller;
}

/**
 * @brief 设置电机控制器输出极性
 * @param polarity 电机控制器输出极性
 */
void Motor::setControllerOutputPolarity(bool polarity)
{
    m_controllerOutputPolarity = polarity;
}

/**
 * @brief 电机开环控制
 * @param controlValue 控制值
 */
void Motor::openloopControl(fp32 controlValue)
{
    m_controllerOutput = controlValue;
    convertControllerOutputToMotorControlData();
}

/**
 * @brief 角度闭环控制
 * @return fp32 控制器输出
 * @note 可使用串级PID，反馈数据数组顺序为[角度误差, 角速度]
 */
fp32 Motor::angleClosedloopControl()
{
    fp32 angleError      = GSRLMath::normalizeDeltaAngle(m_currentAngle - m_targetAngle);
    fp32 feedBackData[2] = {angleError, m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(0.0f, feedBackData, 2);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 角度闭环控制
 * @param targetAngle 目标角度 单位rad [0, 2PI)
 * @return fp32 控制器输出
 */
fp32 Motor::angleClosedloopControl(fp32 targetAngle)
{
    m_targetAngle = targetAngle;
    return angleClosedloopControl();
}

/**
 * @brief 角速度闭环控制
 * @return fp32 控制器输出
 */
fp32 Motor::angularVelocityClosedloopControl()
{
    fp32 feedBackData[1] = {m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(m_targetAngularVelocity, feedBackData, 1);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 角速度闭环控制
 * @param targetAngularVelocity 目标角速度 单位rad/s
 * @return fp32 控制器输出
 */
fp32 Motor::angularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    return angularVelocityClosedloopControl();
}

/**
 * @brief 旋转圈数闭环控制
 * @return fp32 控制器输出
 * @note 可使用串级PID，反馈数据数组顺序为[当前多圈圈数, 角速度]
 */
fp32 Motor::revolutionsClosedloopControl()
{
    fp32 feedBackData[2] = {m_currentRevolutions, m_currentAngularVelocity};
    m_controllerOutput   = m_controller->controllerCalculate(m_targetRevolutions, feedBackData, 2);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 旋转圈数闭环控制
 * @param targetRevolutions 目标旋转圈数
 * @return fp32 控制器输出
 */
fp32 Motor::revolutionsClosedloopControl(fp32 targetRevolutions)
{
    m_targetRevolutions = targetRevolutions;
    return revolutionsClosedloopControl();
}

/**
 * @brief 力矩电流闭环控制
 * @return int16_t 控制器输出
 */
int16_t Motor::torqueCurrentClosedloopControl()
{
    fp32 feedBackData[1] = {(fp32)m_currentTorqueCurrent};
    m_controllerOutput   = m_controller->controllerCalculate((fp32)m_targetTorqueCurrent, feedBackData, 1);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 力矩电流闭环控制
 * @param targetTorqueCurrent 目标力矩电流
 * @return int16_t 控制器输出
 */
int16_t Motor::torqueCurrentClosedloopControl(int16_t targetTorqueCurrent)
{
    m_targetTorqueCurrent = targetTorqueCurrent;
    return torqueCurrentClosedloopControl();
}

/**
 * @brief 外部反馈数据闭环控制
 * @param setPoint 目标值
 * @param feedBackData 反馈数据数组
 * @param feedBackSize 反馈数据数组大小
 */
fp32 Motor::externalClosedloopControl(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize)
{
    m_controllerOutput = m_controller->controllerCalculate(setPoint, feedBackData, feedBackSize);
    if (m_controllerOutputPolarity) {
        m_controllerOutput = -m_controllerOutput;
    }
    convertControllerOutputToMotorControlData();
    return m_controllerOutput;
}

/**
 * @brief 电机基类构造函数，用于初始化电机基本参数
 * @param canControlID 电机CAN控制ID
 * @param canFeedbackID 电机CAN反馈ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @note 该构造函数不可被直接调用，需由派生类构造函数调用
 */
Motor::Motor(uint32_t canControlID, uint32_t canFeedbackID, Controller *controller, uint16_t encoderOffset)
    : m_motorControlMessageID(canControlID),
      m_motorFeedbackMessageID(canFeedbackID),
      m_motorFeedbackSequence(0),
      m_motorLastFeedbackSequence(0),
      m_currentAngle(0.0f), m_lastAngle(0.0f),
      m_currentAngularVelocity(0.0f),
      m_roundCount(0),
      m_zeroAngle(0.0f),
      m_currentRevolutions(0.0f),
      m_currentTorqueCurrent(0),
      m_temperature(0),
      m_motorFeedbackErrorCount(0),
      m_isMotorConnected(false),
      m_targetAngle(0.0f),
      m_targetAngularVelocity(0.0f),
      m_targetRevolutions(0.0f),
      m_targetTorqueCurrent(0),
      m_controller(controller),
      m_controllerOutput(0.0f),
      m_controllerOutputPolarity(false),
      m_encoderOffset(encoderOffset)
{
    m_motorControlHeader.DLC   = 8;
    m_motorControlHeader.IDE   = CAN_ID_STD;
    m_motorControlHeader.RTR   = CAN_RTR_DATA;
    m_motorControlHeader.StdId = m_motorControlMessageID;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
    memset(m_motorFeedbackData, 0, sizeof(m_motorFeedbackData));
}

/**
 * @brief 更新电机当前旋转圈数
 * @return fp32 电机当前旋转圈数
 * @note 该函数调用周期应小于电机旋转周期的一半
 * @note 内部函数，电机接收到数据后会自动调用
 */
fp32 Motor::updateCurrentRevolutions()
{
    if (m_currentAngle - m_lastAngle > MATH_PI) {
        m_roundCount--;
    } else if (m_currentAngle - m_lastAngle < -MATH_PI) {
        m_roundCount++;
    }
    m_currentRevolutions = m_roundCount + (m_currentAngle - m_zeroAngle) / (2 * MATH_PI);
    m_lastAngle          = m_currentAngle;
    return m_currentRevolutions;
}

/**
 * @brief 增加电机反馈错误计数，同时更新电机连接状态
 * @note 电机反馈错误计数超过2则认为电机连接异常
 * @note 该函数仅供内部或派生类调用
 */
inline void Motor::increaseMotorFeedbackErrorCount()
{
    if (m_motorFeedbackErrorCount < 255) {
        m_motorFeedbackErrorCount++;
    }
    m_isMotorConnected = m_motorFeedbackErrorCount < 10;
}

/**
 * @brief 清除电机反馈错误计数，同时更新电机连接状态
 * @note 该函数仅供内部或派生类调用
 */
inline void Motor::clearMotorFeedbackErrorCount()
{
    m_motorFeedbackErrorCount = 0;
    m_isMotorConnected        = true;
}

/******************************************************************************
 *                           GM6020电机类实现
 ******************************************************************************/

/**
 * @brief GM6020电机类构造函数, 用于初始化GM6020电机参数
 * @param dji6020MotorID GM6020电机ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @note GM6020电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |
 * | 反馈ID | 0x205 | 0x206 | 0x207 | 0x208 | 0x209 | 0x20A | 0x20B |
 * | 控制ID |             0x1FF             |         0x2FF         |
 */
MotorGM6020::MotorGM6020(uint8_t dji6020MotorID, Controller *controller, uint16_t encoderOffset)
    : Motor(dji6020MotorID < 5 ? 0x1FF : 0x2FF,
            dji6020MotorID + 0x204,
            controller,
            encoderOffset),
      m_djiMotorID(dji6020MotorID) {}

/**
 * @brief 将控制器输出转换为GM6020电机CAN控制数据
 */
void MotorGM6020::convertControllerOutputToMotorControlData()
{
    int16_t giveControlValue = (int16_t)m_controllerOutput; // 给定控制量, 根据驱动版本可能是电压或电流
    if (m_djiMotorID < 5) {
        m_motorControlData[m_djiMotorID * 2 - 2] = giveControlValue >> 8;
        m_motorControlData[m_djiMotorID * 2 - 1] = giveControlValue;
    } else {
        m_motorControlData[m_djiMotorID * 2 - 10] = giveControlValue >> 8;
        m_motorControlData[m_djiMotorID * 2 - 9]  = giveControlValue;
    }
}

/**
 * @brief 解析GM6020电机CAN反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorGM6020::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    m_encoderHistory[1]      = m_encoderHistory[0];
    m_encoderHistory[0]      = (uint16_t)((((rxMessage.data[0] << 8) | rxMessage.data[1]) - m_encoderOffset) & 0x1FFF);
    m_currentAngle           = (fp32)m_encoderHistory[0] * 2 * MATH_PI / 8192;
    m_currentRPMSpeed        = (int16_t)((rxMessage.data[2] << 8) | rxMessage.data[3]);
    m_currentAngularVelocity = (fp32)m_currentRPMSpeed * 2 * MATH_PI / 60;
    m_currentTorqueCurrent   = (int16_t)((rxMessage.data[4] << 8) | rxMessage.data[5]);
    m_temperature            = rxMessage.data[6];
    return true;
}

/**
 * @brief 获取大疆电机ID
 * @return uint8_t 大疆电机ID
 */
uint8_t MotorGM6020::getDjiMotorID() const
{
    return m_djiMotorID;
}

/**
 * @brief 合并两个同控制ID的大疆电机CAN控制数据, 同时触发双方掉线检测
 * @param otherMotor 另一个同控制ID的大疆电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 * @note 返回的指针指向内部静态缓冲区, 下次调用会覆盖
 */
const uint8_t *MotorGM6020::getMergedControlData(MotorGM6020 &otherMotor)
{
    const uint8_t *selfData  = this->getMotorControlData();
    const uint8_t *otherData = otherMotor.getMotorControlData();

    if (m_motorControlMessageID != otherMotor.m_motorControlMessageID) {
        return selfData;
    }

    memcpy(m_mergedData, selfData, 8);
    uint8_t offset           = (uint8_t)(((otherMotor.m_djiMotorID - 1) & 0x3) * 2);
    m_mergedData[offset]     = otherData[offset];
    m_mergedData[offset + 1] = otherData[offset + 1];
    return m_mergedData;
}

/******************************************************************************
 *                            M3508电机类实现
 ******************************************************************************/

/**
 * @brief M3508电机类构造函数, 用于初始化M3508电机参数
 * @param dji3508MotorID M3508电机ID
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量，默认为0
 * @param gearboxRatio 电机减速比倒数，用于换算电机转速、角速度和圈数，
 *                     电机角度不受此值影响，默认为1即不换算
 * @note M3508电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |
 * | 反馈ID | 0x201 | 0x202 | 0x203 | 0x204 | 0x205 | 0x206 | 0x207 |
 * | 控制ID |             0x200             |         0x1FF         |
 */
MotorM3508::MotorM3508(uint8_t dji3508MotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio)
    : MotorGM6020(dji3508MotorID, controller, encoderOffset),
      m_gearboxRatio(gearboxRatio)
{
    // 调用GM6020构造函数后修正为M3508电机发送和接收ID
    m_motorControlMessageID    = dji3508MotorID < 5 ? 0x200 : 0x1FF;
    m_motorFeedbackMessageID   = dji3508MotorID + 0x200;
    m_motorControlHeader.StdId = m_motorControlMessageID;
}

/**
 * @brief 解析M3508电机反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorM3508::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    // 调用GM6020解析函数后修正M3508电机转速和角速度
    if (MotorGM6020::decodeCanRxMessage(rxMessage)) {
        m_currentRPMSpeed /= m_gearboxRatio;
        m_currentAngularVelocity /= m_gearboxRatio;
        return true;
    }
    return false;
}

/******************************************************************************
 *                           DMJ4310电机类实现
 ******************************************************************************/

/**
 * @brief 达妙DMJ4310电机类构造函数, 用于初始化DMJ4310电机参数
 * @param dmControlID DMJ4310电机控制ID
 * @param dmMasterID DMJ4310电机反馈ID
 * @param pmax 电机最大位置
 * @param vmax 电机最大速度
 * @param tmax 电机最大力矩
 * @param controller 电机绑定控制器
 * @note PMAX、VMAX、TMAX需要与电机固件参数一致，PMAX设置为PI(3.141593)
 */
MotorDM4310::MotorDM4310(uint8_t dmControlID, uint8_t dmMasterID, fp32 pmax, fp32 vmax, fp32 tmax, Controller *controller)
    : Motor(dmControlID, dmMasterID, controller, 0),
      m_motorState(DISABLE),
      m_dmEncoderPosition(0.0f),
      m_mosfetTemperature(0.0f),
      m_coliTemperature(0.0f),
      m_setZeroPositionFlag(false),
      PMAX(pmax),
      VMAX(vmax),
      TMAX(tmax) {}

/**
 * @brief 将控制器输出转换为DMJ4310电机CAN控制数据
 * @note 在转换控制数据前会根据需求先转换特殊电机控制指令如清除错误、使能电机、设置零点
 */
void MotorDM4310::convertControllerOutputToMotorControlData()
{
    // 清除电机失联错误
    if (m_motorState == COMMUNICATION_LOST) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFB;
        return;
    }
    // 使能电机
    if (m_motorState == DISABLE) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFC;
        return;
    }
    // 设置电机零点
    if (m_setZeroPositionFlag) {
        memset(m_motorControlData, 0xFF, sizeof(m_motorControlData));
        m_motorControlData[7] = 0xFE;
        m_setZeroPositionFlag = false;
        return;
    }
    // 电机控制
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp               = GSRLMath::convertFloatToUint(0.0f, -PMAX, PMAX, 16);
    vel_tmp               = GSRLMath::convertFloatToUint(0.0f, -VMAX, VMAX, 12);
    kp_tmp                = GSRLMath::convertFloatToUint(0.0f, 0.0f, 500.0f, 12);
    kd_tmp                = GSRLMath::convertFloatToUint(0.0f, 0.0f, 5.0f, 12);
    tor_tmp               = GSRLMath::convertFloatToUint(m_controllerOutput, -TMAX, TMAX, 12);
    m_motorControlData[0] = (pos_tmp >> 8);
    m_motorControlData[1] = pos_tmp;
    m_motorControlData[2] = (vel_tmp >> 4);
    m_motorControlData[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    m_motorControlData[4] = kp_tmp;
    m_motorControlData[5] = (kd_tmp >> 4);
    m_motorControlData[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    m_motorControlData[7] = tor_tmp;
}

/**
 * @brief 解析DMJ4310电机反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 */
bool MotorDM4310::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;
    if ((rxMessage.data[0] & 0x0F) != m_motorControlMessageID) return false;

    m_motorState             = (DMMotorState)((rxMessage.data[0]) >> 4);
    int rxRawPosition        = (rxMessage.data[1] << 8) | rxMessage.data[2];
    int rxRawVelocity        = (rxMessage.data[3] << 4) | (rxMessage.data[4] >> 4);
    int rxRawTorque          = ((rxMessage.data[4] & 0xF) << 8) | rxMessage.data[5];
    m_dmEncoderPosition      = GSRLMath::convertUintToFloat(rxRawPosition, -PMAX, PMAX, 16); // (-12.5,12.5)
    m_currentAngle           = GSRLMath::normalizeAngle(m_dmEncoderPosition);                // [0,2PI)
    m_currentAngularVelocity = GSRLMath::convertUintToFloat(rxRawVelocity, -VMAX, VMAX, 12); // (-45.0,45.0)
    m_currentTorqueCurrent   = GSRLMath::convertUintToFloat(rxRawTorque, -TMAX, TMAX, 12);   // (-18.0,18.0)
    m_mosfetTemperature      = (float)(rxMessage.data[6]);
    m_coliTemperature        = (float)(rxMessage.data[7]);
    return true;
}

/**
 * @brief 设置电机反馈位置零点
 */
void MotorDM4310::setMotorZeroPosition()
{
    m_setZeroPositionFlag = true;
}

/******************************************************************************
 *                        达妙一控四固件电机类实现
 ******************************************************************************/

/**
 * @brief 达妙一控四固件电机类构造函数, 用于初始化电机参数
 * @param dmMotorID 达妙电机ID, 取值范围 [1, 8]
 * @param controller 电机绑定控制器
 * @param encoderOffset 电机编码器偏移量, 默认为0
 * @note 达妙一控四电机ID对应关系如下:
 * | 电机ID |   1   |   2   |   3   |   4   |   5   |   6   |   7   |   8   |
 * | 反馈ID | 0x301 | 0x302 | 0x303 | 0x304 | 0x305 | 0x306 | 0x307 | 0x308 |
 * | 控制ID |             0x3FE             |             0x4FE             |
 */
MotorDMmulti::MotorDMmulti(uint8_t dmMotorID, Controller *controller, uint16_t encoderOffset)
    : Motor(dmMotorID <= 4 ? 0x3FE : 0x4FE,
            0x300u + dmMotorID,
            controller,
            encoderOffset),
      m_dmMotorID(dmMotorID),
      m_currentRPMSpeed(0),
      m_errorState(0)
{
    m_encoderHistory[0] = 0;
    m_encoderHistory[1] = 0;
}

/**
 * @brief 将控制器输出转换为达妙一控四电机CAN控制数据
 * @note 数据段为小端字节序, 与大疆GM6020的大端字节序相反
 */
void MotorDMmulti::convertControllerOutputToMotorControlData()
{
    int16_t giveControlValue       = (int16_t)m_controllerOutput; // 控制电流标幺值
    uint8_t offset                 = (uint8_t)(((m_dmMotorID - 1) & 0x3) * 2);
    m_motorControlData[offset]     = (uint8_t)(giveControlValue & 0xFF);        // 低 8 位
    m_motorControlData[offset + 1] = (uint8_t)((giveControlValue >> 8) & 0xFF); // 高 8 位
}

/**
 * @brief 解析达妙一控四电机CAN反馈数据核心函数
 * @param rxMessage CAN接收消息
 * @return true ID匹配, 解析成功
 * @return false ID不匹配, 解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 反馈帧格式: D[0:1]位置(0~8191), D[2:3]速度(RPM*100), D[4:5]扭矩电流(mA), D[6]线圈温度, D[7]错误状态
 */
bool MotorDMmulti::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    m_encoderHistory[1] = m_encoderHistory[0];
    m_encoderHistory[0] = (uint16_t)((((rxMessage.data[0] << 8) | rxMessage.data[1]) - m_encoderOffset) & 0x1FFF);
    m_currentAngle      = (fp32)m_encoderHistory[0] * 2.0f * MATH_PI / 8192.0f;

    int16_t rawSpeed100      = (int16_t)((rxMessage.data[2] << 8) | rxMessage.data[3]);
    m_currentRPMSpeed        = (int16_t)(rawSpeed100 / 100);                          // 真实RPM
    m_currentAngularVelocity = (fp32)rawSpeed100 * (2.0f * MATH_PI / 60.0f) / 100.0f; // 保留精度

    m_currentTorqueCurrent = (int16_t)((rxMessage.data[4] << 8) | rxMessage.data[5]); // mA
    m_temperature          = (int8_t)rxMessage.data[6];                               // ℃
    m_errorState           = rxMessage.data[7];
    return true;
}

/**
 * @brief 获取达妙电机ID
 * @return uint8_t 达妙电机ID
 */
uint8_t MotorDMmulti::getDmMotorID() const
{
    return m_dmMotorID;
}

/**
 * @brief 获取达妙电机错误状态
 * @return uint8_t 反馈帧D[7]错误状态字节, 具体含义详见达妙错误状态说明书
 */
uint8_t MotorDMmulti::getErrorState() const
{
    return m_errorState;
}

/**
 * @brief 合并两个同控制ID的一控四固件达妙电机CAN控制数据, 同时触发双方掉线检测
 * @param otherMotor 另一个同控制ID的达妙一控四电机
 * @return const uint8_t* 合并后的8字节CAN控制数据
 * @note 返回的指针指向内部静态缓冲区, 下次调用会覆盖
 */
const uint8_t *MotorDMmulti::getMergedControlData(MotorDMmulti &otherMotor)
{
    const uint8_t *selfData  = this->getMotorControlData();
    const uint8_t *otherData = otherMotor.getMotorControlData();

    if (m_motorControlMessageID != otherMotor.m_motorControlMessageID) {
        return selfData;
    }

    memcpy(m_mergedData, selfData, 8);
    uint8_t offset           = (uint8_t)(((otherMotor.m_dmMotorID - 1) & 0x3) * 2);
    m_mergedData[offset]     = otherData[offset];
    m_mergedData[offset + 1] = otherData[offset + 1];
    return m_mergedData;
}

/******************************************************************************
 *                           瓴控MG系列电机类实现
 ******************************************************************************/

/**
 * @brief 瓴控MG系列电机类构造函数, 用于初始化电机参数
 * @param lkMotorID 瓴控电机ID (控制ID和反馈ID相同)
 * @param controller 绑定的控制器
 * @param encoderOffset 编码器偏移量，默认为0
 * @param gearboxRatio 电机减速比倒数，用于换算电机转速、角速度和圈数，
 *                     电机角度不受此值影响，默认为1即不换算
 * @note lkMotorID范围为1-32，对应反馈ID和控制ID均为0x140 + lkMotorID，Ox140为MG系列电机CAN ID基地址
 */
MotorLKMG::MotorLKMG(uint8_t lkMotorID, Controller *controller, uint16_t encoderOffset, fp32 gearboxRatio)
    : Motor(0x140u + lkMotorID,
            0x140u + lkMotorID,
            controller,
            encoderOffset),
      m_lkMotorID(lkMotorID),
      m_encoderRaw(0),
      m_gearboxRatio(gearboxRatio),
      m_isBraked(true) {}

/**
 * @brief 解析MG电机反馈数据核心函数
 * @param rxMessage CAN接收消息结构体
 * @return true ID匹配，解析成功
 * @return false ID不匹配，解析失败
 * @note 内部函数, 被decodeCanRxMessageFromQueue或decodeCanRxMessageFromISR调用
 * @note 目前只解析0x9C命令，其他命令可根据需要添加
 *       0x9C命令数据格式参考MG系列电机协议文档，为：温度、扭矩电流、速度、原始编码器
 */
bool MotorLKMG::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    if (rxMessage.header.StdId != m_motorFeedbackMessageID) return false;

    uint8_t cmd = rxMessage.data[0];

    switch (cmd) {
        case 0x9C:
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xAD:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8: {
            m_temperature                = (int8_t)rxMessage.data[1];
            m_currentTorqueCurrent       = (int16_t)((rxMessage.data[3] << 8) | rxMessage.data[2]);
            int16_t speedDegreePerSecond = (int16_t)((rxMessage.data[5] << 8) | rxMessage.data[4]);
            m_currentAngularVelocity     = (fp32)speedDegreePerSecond * (MATH_PI / 180.0f) / m_gearboxRatio;
            m_encoderRaw                 = (uint16_t)((rxMessage.data[7] << 8) | rxMessage.data[6]);
            uint16_t encAdj              = (uint16_t)(m_encoderRaw - m_encoderOffset);
            m_currentAngle               = (fp32)encAdj * 2.0f * MATH_PI / (fp32)m_encoderResolution;
        } break;
        case 0x8C: {
            m_isBraked = !(bool)rxMessage.data[1];
        } break;
        // 有需要再根据协议添加解析其他命令
        default:
            return false;
    }
    return true;
}

/**
 * @brief 将控制器输出转换为MG系列电机CAN控制数据(力矩闭环)
 */
void MotorLKMG::convertControllerOutputToMotorControlData()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int16_t iqControl = (int16_t)m_controllerOutput;
        GSRLMath::constrain(iqControl, m_openloopLimit); // 控制量范围 -2048~2048 对应协议
        m_motorControlData[0] = 0xA1;                    // 命令字节：转矩闭环
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = (uint8_t)(iqControl & 0xFF);        // 低字节
        m_motorControlData[5] = (uint8_t)((iqControl >> 8) & 0xFF); // 高字节
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    }
}

/**
 * @brief 电机硬件角速度闭环控制
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t speedControl  = (int32_t)(m_targetAngularVelocity * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *speedBytes   = (uint8_t *)&speedControl;
        m_motorControlData[0] = 0xA2;
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = speedBytes[0];
        m_motorControlData[5] = speedBytes[1];
        m_motorControlData[6] = speedBytes[2];
        m_motorControlData[7] = speedBytes[3];
    }
}

/**
 * @brief 电机硬件角速度闭环控制
 * @param targetAngularVelocity 目标角速度，单位rad/s
 * @note 正反转由符号决定
 */
void MotorLKMG::hardwareAngularVelocityClosedloopControl(fp32 targetAngularVelocity)
{
    m_targetAngularVelocity = targetAngularVelocity;
    hardwareAngularVelocityClosedloopControl();
}

/**
 * @brief 电机硬件单圈角度闭环控制
 */
void MotorLKMG::hardwareAngleClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t targetAngleControl = (int32_t)(m_targetAngle * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *angleBytes        = (uint8_t *)&targetAngleControl;
        float maxVelocityDps       = m_maxVelocity;
        int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *velocityBytes     = (uint8_t *)&maxVelocityControl;
        m_motorControlData[0]      = 0xA6;
        m_motorControlData[1]      = m_isMotorClockwise;
        m_motorControlData[2]      = velocityBytes[0];
        m_motorControlData[3]      = velocityBytes[1];
        m_motorControlData[4]      = angleBytes[0];
        m_motorControlData[5]      = angleBytes[1];
        m_motorControlData[6]      = angleBytes[2];
        m_motorControlData[7]      = angleBytes[3];
    }
}

/**
 * @brief 电机硬件单圈角度闭环控制
 * @param targetAngle 目标角度，单位rad [0, 2PI)
 * @param maxVelocity 最大允许角速度，单位rad/s
 * @param isMotorClockwise 是否顺时针旋转到目标角度，true表示顺时针，false表示逆时针
 */
void MotorLKMG::hardwareAngleClosedloopControl(fp32 targetAngle, fp32 maxVelocity, bool isMotorClockwise)
{
    if (m_isBraked) {
        setBrake(false);
    }
    m_targetAngle      = targetAngle;
    m_maxVelocity      = maxVelocity;
    m_isMotorClockwise = !isMotorClockwise;
    hardwareAngleClosedloopControl();
}

/**
 * @brief 电机硬件多圈角度闭环控制
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl()
{
    if (m_isBraked) {
        setBrake(false);
    } else {
        int32_t targetAngleControl = (int32_t)(m_targetAngle * 100.0f * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *angleBytes        = (uint8_t *)&targetAngleControl;
        float maxVelocityDps       = m_maxVelocity;
        int32_t maxVelocityControl = (int32_t)(maxVelocityDps * m_gearboxRatio * (180.0f / MATH_PI)); // 转换为电机轴速度，单位0.01度每秒
        uint8_t *velocityBytes     = (uint8_t *)&maxVelocityControl;
        m_motorControlData[0]      = 0xA4;
        m_motorControlData[1]      = 0x00;
        m_motorControlData[2]      = velocityBytes[0];
        m_motorControlData[3]      = velocityBytes[1];
        m_motorControlData[4]      = angleBytes[0];
        m_motorControlData[5]      = angleBytes[1];
        m_motorControlData[6]      = angleBytes[2];
        m_motorControlData[7]      = angleBytes[3];
    }
}

/**
 * @brief 电机硬件多圈角度闭环控制
 * @param targetAngle 目标角度，无限制，单位rad
 * @param maxVelocity 最大允许角速度，单位rad/s
 */
void MotorLKMG::hardwareRevolutionsClosedloopControl(fp32 targetAngle, fp32 maxVelocity)
{
    m_targetAngle = targetAngle;
    m_maxVelocity = maxVelocity;
    revolutionsClosedloopControl();
}

/**
 * @brief 设置MG电机刹车(抱闸器)状态
 * @param isBraked 是否刹车，true表示刹车，false表示释放刹车
 */
void MotorLKMG::setBrake(bool isBraked)
{
    if (isBraked) {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x00;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    } else {
        m_motorControlData[0] = 0x8C;
        m_motorControlData[1] = 0x01;
        m_motorControlData[2] = 0x00;
        m_motorControlData[3] = 0x00;
        m_motorControlData[4] = 0x00;
        m_motorControlData[5] = 0x00;
        m_motorControlData[6] = 0x00;
        m_motorControlData[7] = 0x00;
    }
}

/******************************************************************************
 *                          云深处 J60 关节电机类实现
 ******************************************************************************/

/**
 * @brief 云深处 J60 关节电机构造函数
 * @param jointID 关节 ID (1~15)，需与 J60 调试工具中设置的 ID 一致
 * @param controller 电机绑定的控制器 (板外 PID 模式下使用；若仅使用 hardwareMitControl 可传 nullptr)
 * @note CAN 控制 ID 与反馈 ID 均按协议 (cmdIdx<<5) | jointField 动态生成，
 *       基类存储的 ID 仅作 CONTROL 帧的"默认值"，实际发送帧的 StdId 会在状态机切换时更新。
 */
MotorDeepJ60::MotorDeepJ60(uint8_t jointID, Controller *controller)
    : Motor(makeCanId(jointID, CMD_CONTROL),
            makeCanId((uint8_t)(jointID + 0x10), CMD_CONTROL),
            controller,
            0),
      m_jointID(jointID),
      m_state(J60_DISABLED),
      m_intent(INTENT_ENABLE),
      m_currentTorqueNm(0.0f),
      m_mosfetTemperature(0),
      m_motorTemperature(0),
      m_pendingErrorReset(false),
      m_useHardwareMitMode(false),
      m_mitTargetPosition(0.0f),
      m_mitTargetVelocity(0.0f),
      m_mitTargetTorque(0.0f),
      m_mitKp(0.0f),
      m_mitKd(0.0f)
{
    // 基类构造函数已将 DLC=8、IDE=STD、RTR=DATA 设置完毕。
    // 首次 convertControllerOutputToMotorControlData() 会按 m_state 构建 ENABLE 帧。
}

/**
 * @brief 将用户意图置为使能
 * @note 仅修改意图；实际 ENABLE 帧在下一次 convertControllerOutputToMotorControlData() 中发送。
 */
void MotorDeepJ60::enable()
{
    m_intent = INTENT_ENABLE;
}

/**
 * @brief 将用户意图置为失能
 * @note 仅修改意图；实际 DISABLE 帧在下一次 convertControllerOutputToMotorControlData() 中发送。
 */
void MotorDeepJ60::disable()
{
    m_intent = INTENT_DISABLE;
}

/**
 * @brief 请求发送清错命令
 * @note 发送完 ERROR_RESET 后，驱动器回到失能态，m_state 下一周期会被重置为 J60_DISABLED，
 *       若 m_intent 仍为 INTENT_ENABLE，会自动重新 ENABLE。
 * @note 为简化状态机，本方法通过复用 buildErrorResetFrame 在下一次 convert 时插入一帧，
 *       使用独立标志位触发一次性发送。
 */
void MotorDeepJ60::clearError()
{
    // 复位硬件状态为 DISABLED，下一次 convert 会先发 ERROR_RESET 再按 intent 走 ENABLE 流程。
    // 这里直接构造一次 ERROR_RESET header，由 convert 中的状态判断兜底。
    m_pendingErrorReset = true;
    m_state             = J60_DISABLED;
}

/**
 * @brief 硬件 MIT 模式控制 (板载 PD)
 * @param targetPositionRad 目标位置 (rad, [-40, 40])
 * @param targetVelocityRadps 目标速度 (rad/s, [-40, 40])
 * @param targetTorqueNm 前馈扭矩 (Nm, [-40, 40])
 * @param kp 位置环刚度 ([0, 1023])
 * @param kd 速度环阻尼 ([0, 51])
 * @note 调用后控制帧由本方法参数生成，直到 exitHardwareMitMode() 被调用才恢复板外 PID 模式。
 * @note 电机板执行的控制律为 T = Kp*(pDes - pFb) + Kd*(vDes - vFb) + tFF。
 */
void MotorDeepJ60::hardwareMitControl(fp32 targetPositionRad, fp32 targetVelocityRadps,
                                      fp32 targetTorqueNm, fp32 kp, fp32 kd)
{
    m_useHardwareMitMode = true;
    m_mitTargetPosition  = targetPositionRad;
    m_mitTargetVelocity  = targetVelocityRadps;
    m_mitTargetTorque    = targetTorqueNm;
    m_mitKp              = kp;
    m_mitKd              = kd;
    convertControllerOutputToMotorControlData();
}

/**
 * @brief 退出硬件 MIT 模式，恢复使用基类 Controller 作为力矩源
 */
void MotorDeepJ60::exitHardwareMitMode()
{
    m_useHardwareMitMode = false;
    m_mitTargetPosition  = 0.0f;
    m_mitTargetVelocity  = 0.0f;
    m_mitTargetTorque    = 0.0f;
    m_mitKp              = 0.0f;
    m_mitKd              = 0.0f;
}

/**
 * @brief 构造 ENABLE 帧 (cmdIdx=2, DLC=0)
 */
void MotorDeepJ60::buildEnableFrame()
{
    m_motorControlHeader.StdId = makeCanId(m_jointID, CMD_ENABLE);
    m_motorControlHeader.DLC   = 0;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
}

/**
 * @brief 构造 DISABLE 帧 (cmdIdx=1, DLC=0)
 */
void MotorDeepJ60::buildDisableFrame()
{
    m_motorControlHeader.StdId = makeCanId(m_jointID, CMD_DISABLE);
    m_motorControlHeader.DLC   = 0;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
}

/**
 * @brief 构造 ERROR_RESET 帧 (cmdIdx=17, DLC=0)
 */
void MotorDeepJ60::buildErrorResetFrame()
{
    m_motorControlHeader.StdId = makeCanId(m_jointID, CMD_ERROR_RESET);
    m_motorControlHeader.DLC   = 0;
    memset(m_motorControlData, 0, sizeof(m_motorControlData));
}

/**
 * @brief 构造 CONTROL 帧 (cmdIdx=4, DLC=8, MIT 单帧控制)
 * @param pRad 目标位置 (rad)
 * @param vRadps 目标速度 (rad/s)
 * @param tNm 目标扭矩 (Nm)
 * @param kp 刚度 ([0, 1023])
 * @param kd 阻尼 ([0, 51])
 * @note 按协议小端位序：pos[0:15]|vel[16:29]|kp[30:39]|kd[40:47]|tor[48:63]，
 *       通过 uint64_t 拼装后 memcpy 到 CAN 数据区 (STM32 为小端架构)。
 */
void MotorDeepJ60::buildControlFrame(fp32 pRad, fp32 vRadps, fp32 tNm, fp32 kp, fp32 kd)
{
    m_motorControlHeader.StdId = makeCanId(m_jointID, CMD_CONTROL);
    m_motorControlHeader.DLC   = 8;

    // 限幅到协议允许的物理量范围
    GSRLMath::constrain(pRad, J60_POSITION_MIN, J60_POSITION_MAX);
    GSRLMath::constrain(vRadps, J60_VELOCITY_MIN, J60_VELOCITY_MAX);
    GSRLMath::constrain(tNm, J60_TORQUE_MIN, J60_TORQUE_MAX);
    GSRLMath::constrain(kp, 0.0f, J60_KP_MAX);
    GSRLMath::constrain(kd, 0.0f, J60_KD_MAX);

    uint64_t pos = (uint64_t)GSRLMath::convertFloatToUint(pRad, J60_POSITION_MIN, J60_POSITION_MAX, 16);
    uint64_t vel = (uint64_t)GSRLMath::convertFloatToUint(vRadps, J60_VELOCITY_MIN, J60_VELOCITY_MAX, 14);
    uint64_t kpI = (uint64_t)GSRLMath::convertFloatToUint(kp, 0.0f, J60_KP_MAX, 10);
    uint64_t kdI = (uint64_t)GSRLMath::convertFloatToUint(kd, 0.0f, J60_KD_MAX, 8);
    uint64_t tor = (uint64_t)GSRLMath::convertFloatToUint(tNm, J60_TORQUE_MIN, J60_TORQUE_MAX, 16);

    uint64_t payload = 0;
    payload |= (pos & 0xFFFFull) << 0;  // bit  0-15
    payload |= (vel & 0x3FFFull) << 16; // bit 16-29
    payload |= (kpI & 0x3FFull) << 30;  // bit 30-39
    payload |= (kdI & 0xFFull) << 40;   // bit 40-47
    payload |= (tor & 0xFFFFull) << 48; // bit 48-63

    memcpy(m_motorControlData, &payload, sizeof(payload));
}

/**
 * @brief 根据状态机与用户意图构造当前应发送的帧
 * @note 状态转移矩阵：
 *       | intent   | hwState  | 发送帧       |
 *       | -------- | -------- | ----------- |
 *       | ENABLE   | DISABLED | ENABLE      |
 *       | ENABLE   | ENABLED  | CONTROL     |
 *       | DISABLE  | *        | DISABLE     |
 * @note 掉线检测：若基类判定失联，则把 m_state 重置为 J60_DISABLED 以触发自动重连。
 */
void MotorDeepJ60::convertControllerOutputToMotorControlData()
{
    // 掉线探测：若基类已判定失联，则重置硬件状态，下一帧起重发 ENABLE。
    if (!m_isMotorConnected) {
        m_state = J60_DISABLED;
    }

    // 一次性 ERROR_RESET 帧插入 (clearError() 设置)
    if (m_pendingErrorReset) {
        m_pendingErrorReset = false;
        buildErrorResetFrame();
        return;
    }

    if (m_intent == INTENT_DISABLE) {
        buildDisableFrame();
        return;
    }

    // m_intent == INTENT_ENABLE
    if (m_state == J60_DISABLED) {
        buildEnableFrame();
        return;
    }

    // m_intent == INTENT_ENABLE && m_state == J60_ENABLED → 发送控制帧
    if (m_useHardwareMitMode) {
        buildControlFrame(m_mitTargetPosition, m_mitTargetVelocity,
                          m_mitTargetTorque, m_mitKp, m_mitKd);
    } else {
        // 板外 PID 模式：Kp=Kd=0，位置/速度目标置零 (不参与板载 PD)，扭矩前馈由 m_controllerOutput 提供。
        // 基类的 angleClosedloopControl / torqueCurrentClosedloopControl 等已处理 m_controllerOutputPolarity。
        buildControlFrame(0.0f, 0.0f, m_controllerOutput, 0.0f, 0.0f);
    }
}

/**
 * @brief 解析 J60 反馈 CAN 帧
 * @param rxMessage CAN 接收消息
 * @return true ID 属于本关节且命令索引已识别，解析成功
 * @return false ID 不属于本关节或命令索引未支持
 * @note J60 反馈 ID 的 Bit0~Bit4 = jointID + 0x10 (驱动器应答时 bit4 置 1)，
 *       Bit5~Bit10 = 命令索引。本函数按位段精匹配，避免与其它电机 ID 冲突。
 * @note CONTROL 响应帧 (DLC=8) 的位段 (小端)：
 *       pos[0:19]|vel[20:39]|tor[40:55]|tempFlag[56]|temp[57:63]
 */
bool MotorDeepJ60::decodeCanRxMessage(const can_rx_message_t &rxMessage)
{
    uint16_t rxId         = (uint16_t)rxMessage.header.StdId;
    uint8_t rxJointField  = (uint8_t)(rxId & 0x1Fu);
    uint8_t rxCmdIndex    = (uint8_t)((rxId >> 5) & 0x3Fu);
    uint8_t expectedField = (uint8_t)(m_jointID + 0x10u);

    if (rxJointField != expectedField) {
        return false;
    }

    switch (rxCmdIndex) {
        case CMD_ENABLE: {
            // 使能应答 DLC=1, byte0=0 表示成功
            if (rxMessage.header.DLC >= 1 && rxMessage.data[0] == 0) {
                m_state = J60_ENABLED;
            }
            return true;
        }
        case CMD_DISABLE:
        case CMD_ERROR_RESET: {
            // 失能成功 / 清错成功后驱动器均回到失能态，由后续 ENABLE 流程恢复
            if (rxMessage.header.DLC >= 1 && rxMessage.data[0] == 0) {
                m_state = J60_DISABLED;
            }
            return true;
        }
        case CMD_CONTROL: {
            if (rxMessage.header.DLC != 8) {
                return false;
            }

            // 收到控制应答即证明硬件已使能 (驱动器在失能态下收到控制帧也会回应，但仍可作为 online 证据)
            m_state = J60_ENABLED;

            uint64_t payload = 0;
            memcpy(&payload, rxMessage.data, 8);

            uint32_t posRaw = (uint32_t)((payload >> 0) & 0xFFFFFull);  // 20 bit
            uint32_t velRaw = (uint32_t)((payload >> 20) & 0xFFFFFull); // 20 bit
            uint32_t torRaw = (uint32_t)((payload >> 40) & 0xFFFFull);  // 16 bit
            uint8_t tempFlg = (uint8_t)((payload >> 56) & 0x1ull);
            uint8_t tempRaw = (uint8_t)((payload >> 57) & 0x7Full);

            fp32 decodedPosRad =
                GSRLMath::convertUintToFloat((int)posRaw, J60_POSITION_MIN, J60_POSITION_MAX, 20);
            fp32 decodedVelRadps =
                GSRLMath::convertUintToFloat((int)velRaw, J60_VELOCITY_MIN, J60_VELOCITY_MAX, 20);
            fp32 decodedTorNm =
                GSRLMath::convertUintToFloat((int)torRaw, J60_TORQUE_MIN, J60_TORQUE_MAX, 16);
            fp32 decodedTempC =
                GSRLMath::convertUintToFloat((int)tempRaw, J60_TEMP_MIN, J60_TEMP_MAX, 7);

            m_currentAngle           = GSRLMath::normalizeAngle(decodedPosRad);
            m_currentAngularVelocity = decodedVelRadps;
            m_currentTorqueNm        = decodedTorNm;
            // 兼容基类 getCurrentTorqueCurrent()：以 mNm 存储，钳位到 int16 范围 (±32767)；
            // 精确扭矩请使用 getCurrentTorqueNm()。
            fp32 torqueMnm = decodedTorNm * 1000.0f;
            GSRLMath::constrain(torqueMnm, -32767.0f, 32767.0f);
            m_currentTorqueCurrent = (int16_t)torqueMnm;

            // 温度：J60 专有成员用 int16_t 存完整范围，基类 m_temperature (int8_t) 钳位
            fp32 tempClamped = decodedTempC;
            GSRLMath::constrain(tempClamped, -128.0f, 127.0f);
            if (tempFlg == 1) {
                m_motorTemperature = (int16_t)decodedTempC;
                m_temperature      = (int8_t)tempClamped;
            } else {
                m_mosfetTemperature = (int16_t)decodedTempC;
                m_temperature       = (int8_t)tempClamped;
            }
            return true;
        }
        default:
            return false;
    }
}
