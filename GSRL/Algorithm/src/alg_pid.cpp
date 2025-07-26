/**
 ******************************************************************************
 * @file           : alg_pid.cpp
 * @brief          : PID算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "alg_pid.hpp"
#include "alg_general.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            SimplePID类实现
 ******************************************************************************/

/**
 * @brief SimplePID构造函数
 * @param mode PID模式枚举值
 * @param param PID参数结构体
 * @param filter 反馈值滤波器指针, 默认为nullptr即不使用滤波器
 * @note 初始化PID控制器,设置PID参数并重置PID
 */
SimplePID::SimplePID(PIDMode mode, PIDParam &param, Filter<fp32> *filter)
    : m_mode(mode), m_param(param), m_filter(filter)
{
    pidClear();
}

/**
 * @brief pid计算
 * @param setPoint 设定值
 * @param feedBackData 反馈值数组
 * @param feedBackSize 反馈值数组长度
 * @return PID输出值
 * @note 取反馈值数组的首个值进行计算
 * @note 如果设置了滤波器, 则对反馈值进行滤波后再进行计算
 */
fp32 SimplePID::controllerCalculate(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize)
{
    fp32 filterData;
    if (m_filter != nullptr) {
        filterData = m_filter->filterCalculate(feedBackData[0]);
    } else {
        filterData = feedBackData[0];
    }
    m_data.errorHistory[2] = m_data.errorHistory[1];
    m_data.errorHistory[1] = m_data.errorHistory[0];
    m_data.setPoint        = setPoint;
    m_data.feedBackData    = filterData; // 使用第一个反馈值
    m_data.errorHistory[0] = setPoint - filterData;
    if (m_mode == PID_POSITION) {
        m_data.pOut = m_param.Kp * m_data.errorHistory[0];
        m_data.iOut += m_param.Ki * m_data.errorHistory[0];
        m_data.derivativeHistory[2] = m_data.derivativeHistory[1];
        m_data.derivativeHistory[1] = m_data.derivativeHistory[0];
        m_data.derivativeHistory[0] = (m_data.errorHistory[0] - m_data.errorHistory[1]);
        m_data.dOut                 = m_param.Kd * m_data.derivativeHistory[0];
        GSRLMath::constrain(m_data.iOut, m_param.intergralLimit);
        m_data.output = m_data.pOut + m_data.iOut + m_data.dOut;
        GSRLMath::constrain(m_data.output, m_param.outputLimit);
    } else if (m_mode == PID_DELTA) {
        m_data.pOut                 = m_param.Kp * (m_data.errorHistory[0] - m_data.errorHistory[1]);
        m_data.iOut                 = m_param.Ki * m_data.errorHistory[0];
        m_data.derivativeHistory[2] = m_data.derivativeHistory[1];
        m_data.derivativeHistory[1] = m_data.derivativeHistory[0];
        m_data.derivativeHistory[0] = (m_data.errorHistory[0] - 2.0f * m_data.errorHistory[1] + m_data.errorHistory[2]);
        m_data.dOut                 = m_param.Kd * m_data.derivativeHistory[0];
        m_data.output += m_data.pOut + m_data.iOut + m_data.dOut;
        GSRLMath::constrain(m_data.output, m_param.outputLimit);
    }
    return m_data.output;
}

/**
 * @brief 重置PID
 * @details 将PID控制器的历史数据清零，同时重置滤波器
 */
void SimplePID::pidClear()
{
    if (m_filter != nullptr) m_filter->reset();
    m_data.errorHistory[0] = m_data.errorHistory[1] = m_data.errorHistory[2] = 0.0f;
    m_data.derivativeHistory[0] = m_data.derivativeHistory[1] = m_data.derivativeHistory[2] = 0.0f;
    m_data.pOut = m_data.iOut = m_data.dOut = m_data.output = 0.0f;
    m_data.setPoint = m_data.feedBackData = 0.0f;
}

/**
 * @brief 设置PID模式
 * @param mode PID模式枚举值
 */
void SimplePID::pidSetMode(PIDMode mode)
{
    m_mode = mode;
}

/**
 * @brief 获取PID模式
 * @return PID模式枚举值
 */
SimplePID::PIDMode SimplePID::pidGetMode() const
{
    return m_mode;
}

/**
 * @brief 设置PID参数
 * @param param PID参数结构体
 */
void SimplePID::pidSetParam(PIDParam &param)
{
    m_param = param;
}

/**
 * @brief 获取PID参数
 * @return PID参数结构体
 */
SimplePID::PIDParam SimplePID::pidGetParam() const
{
    return m_param;
}

/**
 * @brief 获取PID数据
 * @return PID数据结构体
 */
SimplePID::PIDData SimplePID::pidGetData() const
{
    return m_data;
}

/******************************************************************************
 *                            CascadePID类实现
 ******************************************************************************/

/**
 * @brief CascadePID构造函数
 * @param outerParam 外环PID参数结构体
 * @param innerParam 内环PID参数结构体
 * @param outerFilter 外环滤波器指针, 默认为nullptr即不使用滤波器
 * @param innerFilter 内环滤波器指针, 默认为nullptr即不使用滤波器
 * @details 初始化串级PID控制器,设置内外环PID参数和滤波器并重置PID
 */
CascadePID::CascadePID(PIDParam &outerParam, PIDParam &innerParam, Filter<fp32> *outerFilter, Filter<fp32> *innerFilter)
    : m_outerLoop(SimplePID::PID_POSITION, outerParam, outerFilter),
      m_innerLoop(SimplePID::PID_POSITION, innerParam, innerFilter)
{
}

/**
 * @brief 串级PID计算
 * @param setPoint 设定值
 * @param feedBackData 反馈值数组
 * @param feedBackSize 反馈值数组长度
 * @return PID输出值
 * @note 反馈值数组长度必须大于等于2，否则返回0
 * @note 取反馈值数组的前两个元素，顺序为[外环反馈值, 内环反馈值]进行计算
 */
fp32 CascadePID::controllerCalculate(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize)
{
    if (feedBackSize < 2) return 0.0f;
    fp32 innerSetPoint = m_outerLoop.controllerCalculate(setPoint, &feedBackData[0], 1); // 外环计算
    return m_innerLoop.controllerCalculate(innerSetPoint, &feedBackData[1], 1);          // 内环计算
}

/**
 * @brief 重置串级PID
 * @note 将内外环PID控制器的历史数据和滤波器清零
 */
void CascadePID::cascadeClear()
{
    m_outerLoop.pidClear();
    m_innerLoop.pidClear();
}

/**
 * @brief 获取外环PID控制器
 * @return 外环PID控制器引用
 */
SimplePID &CascadePID::getOuterLoop()
{
    return m_outerLoop;
}

/**
 * @brief 获取内环PID控制器
 * @return 内环PID控制器引用
 */
SimplePID &CascadePID::getInnerLoop()
{
    return m_innerLoop;
}
