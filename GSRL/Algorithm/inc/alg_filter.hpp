/**
 ******************************************************************************
 * @file           : alg_filter.hpp
 * @brief          : header file for alg_filter.cpp
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

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 滤波器接口
 * @details 定义调用滤波器的输入输出接口函数
 */
template <typename T>
class Filter
{
public:
    virtual ~Filter()                  = default;
    virtual T filterCalculate(T input) = 0;
    virtual void reset()               = 0;
};

/**
 * @brief 一阶低通滤波器
 * @details 一阶低通滤波器，可用于平滑信号
 */
template <typename T = fp32>
class LowPassFilter : public Filter<T>
{
private:
    T m_alpha;      // 滤波系数 (0-1)
    T m_lastOutput; // 上一次的输出值

public:
    /**
     * @brief 构造函数
     * @param alpha 滤波系数 (0-1)，越接近1，滤波效果越弱
     */
    LowPassFilter(T alpha = 0.7f)
        : m_alpha(alpha), m_lastOutput(0)
    {
    }

    /**
     * @brief 滤波计算
     * @param input 输入值
     * @return 滤波后的输出值
     */
    T filterCalculate(T input) override
    {
        m_lastOutput = m_alpha * input + (1 - m_alpha) * m_lastOutput;
        return m_lastOutput;
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_lastOutput = 0;
    }

    /**
     * @brief 设置滤波系数
     * @param alpha 滤波系数 (0-1)
     */
    void setAlpha(T alpha)
    {
        m_alpha = alpha;
    }

    /**
     * @brief 获取滤波系数
     * @return 滤波系数
     */
    T getAlpha() const
    {
        return m_alpha;
    }
};

/**
 * @brief 移动平均滤波器
 * @details 使用一个固定大小的窗口计算平均值
 */
template <typename T = fp32, size_t WindowSize = 10>
class MovingAverageFilter : public Filter<T>
{
private:
    T m_buffer[WindowSize]; // 数据缓冲区
    size_t m_head;          // 缓冲区头指针
    size_t m_count;         // 当前缓冲区内的数据数量
    T m_sum;                // 当前所有数据的和

public:
    /**
     * @brief 构造函数
     */
    MovingAverageFilter()
        : m_head(0), m_count(0), m_sum(0)
    {
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }

    /**
     * @brief 滤波计算
     * @param input 输入值
     * @return 滤波后的输出值
     */
    T filterCalculate(T input) override
    {
        // 更新总和，减去将被替换的值
        if (m_count == WindowSize) {
            m_sum -= m_buffer[m_head];
        } else {
            m_count++;
        }

        // 添加新值
        m_buffer[m_head] = input;
        m_sum += input;

        // 更新头指针
        m_head = (m_head + 1) % WindowSize;

        // 计算平均值
        return m_sum / static_cast<T>(m_count);
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_head  = 0;
        m_count = 0;
        m_sum   = 0;
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }

    /**
     * @brief 获取窗口大小
     * @return 窗口大小
     */
    size_t getWindowSize() const
    {
        return WindowSize;
    }

    /**
     * @brief 获取当前缓冲区中的数据数量
     * @return 数据数量
     */
    size_t getCount() const
    {
        return m_count;
    }
};

/**
 * @brief 卡尔曼滤波器
 * @details 一维卡尔曼滤波器，用于从噪声数据中估计真实值
 */
template <typename T = fp32>
class KalmanFilter : public Filter<T>
{
public:
    struct KalmanParam {
        T processNoise;     // 过程噪声方差 Q
        T measurementNoise; // 测量噪声方差 R
    };

private:
    KalmanParam m_param;
    T m_stateEstimate;   // 状态估计值 x
    T m_errorCovariance; // 估计误差协方差 P

public:
    /**
     * @brief 构造函数
     * @param param 卡尔曼滤波器参数
     * @param initialState 初始状态估计
     * @param initialCovariance 初始估计误差协方差
     */
    KalmanFilter(KalmanParam param, T initialState = 0, T initialCovariance = 1)
        : m_param(param), m_stateEstimate(initialState), m_errorCovariance(initialCovariance)
    {
    }

    /**
     * @brief 滤波计算
     * @param input 输入值 (测量值)
     * @return 滤波后的输出值 (估计值)
     */
    T filterCalculate(T input) override
    {
        // 预测步骤
        m_errorCovariance += m_param.processNoise;

        // 更新步骤
        T kalmanGain = m_errorCovariance / (m_errorCovariance + m_param.measurementNoise);
        m_stateEstimate += kalmanGain * (input - m_stateEstimate);
        m_errorCovariance = (1 - kalmanGain) * m_errorCovariance;

        return m_stateEstimate;
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_stateEstimate   = 0;
        m_errorCovariance = 1;
    }

    /**
     * @brief 设置卡尔曼滤波器参数
     * @param param 参数结构体
     */
    void setParam(KalmanParam param)
    {
        m_param = param;
    }

    /**
     * @brief 获取卡尔曼滤波器参数
     * @return 参数结构体
     */
    KalmanParam getParam() const
    {
        return m_param;
    }
};

/**
 * @brief 中值滤波器
 * @details 用于滤除脉冲噪声，保留信号边缘特性
 */
template <typename T = fp32, size_t WindowSize = 3>
class MedianFilter : public Filter<T>
{
private:
    T m_buffer[WindowSize]; // 数据缓冲区
    size_t m_head;          // 缓冲区头指针
    size_t m_count;         // 当前缓冲区内的数据数量

public:
    /**
     * @brief 构造函数
     */
    MedianFilter()
        : m_head(0), m_count(0)
    {
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }

    /**
     * @brief 滤波计算
     * @param input 输入值
     * @return 滤波后的输出值
     */
    T filterCalculate(T input) override
    {
        // 添加新值
        m_buffer[m_head] = input;
        m_head           = (m_head + 1) % WindowSize;
        if (m_count < WindowSize) {
            m_count++;
        }

        // 复制一份数据用于排序
        T sortedBuffer[WindowSize];
        size_t idx = 0;
        size_t pos = (m_head + WindowSize - m_count) % WindowSize; // 找到最旧数据的位置
        for (size_t i = 0; i < m_count; i++) {
            sortedBuffer[idx++] = m_buffer[pos];
            pos                 = (pos + 1) % WindowSize;
        }

        // 简单冒泡排序 (只对有效数据排序)
        for (size_t i = 0; i < m_count - 1; i++) {
            for (size_t j = 0; j < m_count - i - 1; j++) {
                if (sortedBuffer[j] > sortedBuffer[j + 1]) {
                    T temp              = sortedBuffer[j];
                    sortedBuffer[j]     = sortedBuffer[j + 1];
                    sortedBuffer[j + 1] = temp;
                }
            }
        }

        // 返回中值
        return sortedBuffer[m_count / 2];
    }

    /**
     * @brief 重置滤波器
     */
    void reset() override
    {
        m_head  = 0;
        m_count = 0;
        for (size_t i = 0; i < WindowSize; i++) {
            m_buffer[i] = 0;
        }
    }
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
