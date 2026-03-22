/**
 ******************************************************************************
 * @file           : alg_ahrs.hpp
 * @brief          : header file for alg_ahrs.cpp
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
#include "alg_general.hpp"
#include "alg_filter.hpp"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief AHRS姿态解算算法接口
 */
class AHRS
{
protected:
    using Vector3f = GSRLMath::Vector3f;
    Vector3f m_gyro;
    Vector3f m_accel;
    Vector3f m_magnet;
    Vector3f m_eulerAngle;
    fp32 m_quaternion[4];
    Vector3f m_motionAccelBodyFrame;  // 机体坐标系下的运动加速度
    Vector3f m_motionAccelEarthFrame; // 大地坐标系下的运动加速度
    bool m_isAhrsInited;              // AHRS初始化完成标志

public:
    virtual ~AHRS() = default;
    virtual void reset();
    virtual void init() = 0; // AHRS初始化纯虚函数，在第一次update被调用时执行
    const Vector3f &update(const Vector3f &gyro, const Vector3f &accel, const Vector3f &magnet = Vector3f());
    virtual const Vector3f &getGyro() const;
    virtual const Vector3f &getAccel() const;
    virtual const Vector3f &getMotionAccelBodyFrame() const;
    virtual const Vector3f &getMotionAccelEarthFrame() const;
    const fp32 *getQuaternion() const;
    const Vector3f &getEulerAngle() const;

protected:
    AHRS();
    virtual void dataProcess() = 0;
    void convertQuaternionToEulerAngle();
    void initQuaternion();
    void calculateMotionAccel();
};

class Mahony : public AHRS
{
private:
    // 采样频率相关
    fp32 m_sampleFreq;              // sample frequency in Hz
    fp32 m_deltaTime;               // sample period in seconds
    uint32_t m_lastUpdateTimestamp; // DWT counter value of the last update
    // PI补偿器相关
    fp32 m_integralFBx, m_integralFBy, m_integralFBz;
    fp32 m_twoKi, m_twoKp;
    // 加速度低通滤波相关
    Vector3f m_accelFilterHistory[3]; // 0: oldest, 2: newest
    Vector3f m_accelFiltered;
    Vector3f m_accelFilterNum;

public:
    Mahony(fp32 sampleFreq = 0.0f, Vector3f accelFilterNum = 0, fp32 kp = 0.5f, fp32 ki = 0.0f);
    void reset() override;
    void init() override;
    const Vector3f &getAccel() const override;

private:
    void dataProcess() override;
    void sixAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az);
    void nineAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz);
    void filterAccel();
};

/**
 * @brief 四元数扩展卡尔曼滤波(EKF)姿态解算类
 * @details 使用6维状态向量(四元数4维 + 陀螺仪xy轴零偏2维)的EKF算法进行姿态解算,
 *          包含完整的卡方检验自适应机制、发散保护、零偏协方差衰减(fading)、
 *          自适应增益缩放及方向余弦加权等鲁棒性处理。
 *          状态向量: [q0, q1, q2, q3, bias_x, bias_y]
 *          观测向量: 归一化加速度计三轴数据
 */
class QuaternionEKF : public AHRS
{
private:
    // 初始化相关
    using KF = KalmanFilter<fp32, 6, 3, 0>;
    KF m_kalmanFilter; // 卡尔曼滤波器实例
    // 采样频率相关
    fp32 m_sampleFreq;              // 固定采样频率(Hz), 为0则使用DWT自动计算
    uint32_t m_lastUpdateTimestamp; // DWT计数器历史值
    fp32 m_deltaTime;               // 实际采样周期(s)
    // 卡尔曼噪声参数
    fp32 m_quatProcessNoise; // 四元数更新过程噪声基准(Q矩阵)
    fp32 m_biasProcessNoise; // 陀螺仪零偏过程噪声基准(Q矩阵)
    fp32 m_measNoise;        // 加速度计量测噪声基准(R矩阵)
    fp32 m_lambda;           // 衰减系数(fading factor), 防止零偏协方差过度收敛
    // 滤波中间量
    Vector3f m_accelFiltered; // 低通滤波后的加速度值
    Vector3f m_gyroBias;      // 陀螺仪xy轴零偏估计
    // 卡方检测与自适应机制
    fp32 m_chiSquareThreshold;    // 卡方检验阈值
    fp32 m_accLpfCoef;            // 加速度计一阶低通滤波时间常数(s), 0表示不滤波
    bool m_isCheckChiSquare;      // 是否启用卡方检验
    bool m_convergeFlag;          // 滤波器收敛标志
    bool m_stableFlag;            // 载体静止稳定标志(角速度小且加速度接近重力)
    uint32_t m_errorCount;        // 连续卡方检验失败计数(用于发散保护)
    fp32 m_adaptiveGainScale;     // 自适应增益缩放因子
    fp32 m_gyroNorm;              // 当前角速度向量范数
    fp32 m_accelNorm;             // 当前加速度向量范数
    Vector3f m_orientationCosine; // 预测重力方向与各轴的余弦角
public:
    QuaternionEKF(fp32 sampleFreq         = 0.0f,
                  fp32 quatProcessNoise   = 10.0f,
                  fp32 biasProcessNoise   = 0.001f,
                  fp32 measNoise          = 1e6f,
                  fp32 lambda             = 1.0f,
                  fp32 accLpfCoef         = 0.0f,
                  bool isCheckChiSquare   = true,
                  fp32 chiSquareThreshold = 1e-8f);

    void reset() override;
    void init() override;

private:
    void dataProcess() override;
    void ekfProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
