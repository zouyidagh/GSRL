/**
 ******************************************************************************
 * @file           : alg_pid.hpp
 * @brief          : header file for alg_pid.cpp
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
#include "alg_filter.hpp"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 控制器接口
 * @note 定义调用控制器的输入输出接口函数
 */
class Controller
{
public:
    virtual ~Controller() = default;
    virtual fp32 controllerCalculate(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize) = 0;
};

/**
 * @brief 简单PID控制器
 * @note 该PID控制器支持位置式和增量式PID控制，改自大疆官方开源
 */
class SimplePID : public Controller
{
public:
    enum PIDMode : uint8_t {
        PID_POSITION = 0,
        PID_DELTA
    };

    struct PIDParam {
        fp32 Kp;
        fp32 Ki;
        fp32 Kd;
        fp32 outputLimit;
        fp32 intergralLimit;
    };

    struct PIDData {
        fp32 setPoint;
        fp32 feedBackData;
        fp32 output;
        fp32 pOut;
        fp32 iOut;
        fp32 dOut;
        fp32 derivativeHistory[3]; // 微分项 0最新 1上一次 2上上次
        fp32 errorHistory[3];      // 误差项 0最新 1上一次 2上上次
    };

private:
    PIDMode m_mode;
    PIDParam m_param;
    PIDData m_data;
    Filter<fp32> *m_filter; // 滤波器接口

public:
    SimplePID(PIDMode mode, PIDParam &param, Filter<fp32> *filter = nullptr);
    fp32 controllerCalculate(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize = 1) override;
    void pidClear();

    void pidSetMode(PIDMode mode);
    PIDMode pidGetMode() const;
    void pidSetParam(PIDParam &param);
    PIDParam pidGetParam() const;
    PIDData pidGetData() const;
};

/**
 * @brief 串级位置式PID控制器
 * @note 包含内环和外环两个PID控制器，外环输出作为内环输入
 * @note 使用SimplePID作为两环的PID控制器
 */
class CascadePID : public Controller
{
public:
    using PIDParam = SimplePID::PIDParam;
    using PIDData  = SimplePID::PIDData;

private:
    SimplePID m_outerLoop;
    SimplePID m_innerLoop;

public:
    CascadePID(PIDParam &outerParam, PIDParam &innerParam, Filter<fp32> *outerFilter = nullptr, Filter<fp32> *innerFilter = nullptr);
    fp32 controllerCalculate(fp32 setPoint, const fp32 *feedBackData, uint8_t feedBackSize = 2) override;
    void cascadeClear();
    SimplePID &getOuterLoop();
    SimplePID &getInnerLoop();
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
