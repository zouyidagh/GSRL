/**
 ******************************************************************************
 * @file           : alg_ahrs.cpp
 * @brief          : AHRS姿态解算算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "drv_misc.h"
#include "alg_ahrs.hpp"

/* Typedef -------------------------------------------------------------------*/

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            AHRS类实现
 ******************************************************************************/

/**
 * @brief 重置AHRS姿态解算相关数据
 * @note 归零传感器缓冲数据、欧拉角, 重置四元数为[1, 0, 0, 0]
 */
void AHRS::reset()
{
    m_gyro          = 0;
    m_accel         = 0;
    m_magnet        = 0;
    m_eulerAngle    = 0;
    m_quaternion[0] = 1.0f;
    m_quaternion[1] = 0.0f;
    m_quaternion[2] = 0.0f;
    m_quaternion[3] = 0.0f;
}

/**
 * @brief 初始化AHRS姿态解算, 根据加速度计和磁力计数据初始化四元数
 * @param accel 加速度计数据
 * @param magnet 磁力计数据
 */
void AHRS::init(const Vector3f &accel, const Vector3f &magnet)
{
    // 存储传入的加速度和磁力计数据
    m_accel  = accel;
    m_magnet = magnet;

    // 本地变量
    fp32 R[9]; // 3x3旋转矩阵
    fp32 norm, fmodx, fmody;

    // 将未归一化的重力和地磁向量放入旋转矩阵的z轴和x轴
    R[2] = accel.x;
    R[5] = accel.y;
    R[8] = accel.z;
    R[0] = magnet.x;
    R[3] = magnet.y;
    R[6] = magnet.z;

    // 设置y向量为z和x向量的叉乘
    R[1] = R[5] * R[6] - R[8] * R[3];
    R[4] = R[8] * R[0] - R[2] * R[6];
    R[7] = R[2] * R[3] - R[5] * R[0];

    // 设置x向量为y和z向量的叉乘
    R[0] = R[4] * R[8] - R[7] * R[5];
    R[3] = R[7] * R[2] - R[1] * R[8];
    R[6] = R[1] * R[5] - R[4] * R[2];

    // 计算向量模的倒数
    norm  = GSRLMath::invSqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
    fmodx = GSRLMath::invSqrt(R[0] * R[0] + R[3] * R[3] + R[6] * R[6]);
    fmody = GSRLMath::invSqrt(R[1] * R[1] + R[4] * R[4] + R[7] * R[7]);

    // 归一化旋转矩阵
    R[0] *= fmodx;
    R[3] *= fmodx;
    R[6] *= fmodx; // 归一化x轴
    R[1] *= fmody;
    R[4] *= fmody;
    R[7] *= fmody; // 归一化y轴
    R[2] *= norm;
    R[5] *= norm;
    R[8] *= norm; // 归一化z轴

    // ---- 从旋转矩阵计算四元数 ----
    fp32 fq0sq;                     // q0^2
    fp32 recip4q0;                  // 1/4q0
    fp32 fmag;                      // 四元数幅值
    constexpr fp32 SMALLQ0 = 0.01F; // 舍入误差可能出现的限制

    // 获取q0^2和q0
    fq0sq           = 0.25f * (1.0f + R[0] + R[4] + R[8]);
    m_quaternion[0] = sqrtf(fabsf(fq0sq));

    // 正常情况，当q0不小，意味着旋转角度不接近180度
    if (m_quaternion[0] > SMALLQ0) {
        // 计算q1到q3
        recip4q0        = 0.25F / m_quaternion[0];
        m_quaternion[1] = recip4q0 * (R[5] - R[7]);
        m_quaternion[2] = recip4q0 * (R[6] - R[2]);
        m_quaternion[3] = recip4q0 * (R[1] - R[3]);
    } else {
        // 接近180度的特殊情况对应近似对称矩阵
        m_quaternion[1] = sqrtf(fabsf(0.5f * (1.0f + R[0]) - fq0sq));
        m_quaternion[2] = sqrtf(fabsf(0.5f * (1.0f + R[4]) - fq0sq));
        m_quaternion[3] = sqrtf(fabsf(0.5f * (1.0f + R[8]) - fq0sq));

        // 确定四元数各分量符号
        if ((R[1] + R[3]) < 0.0f) {
            m_quaternion[2] = -m_quaternion[2];
            if ((R[5] + R[7]) > 0.0f) {
                m_quaternion[3] = -m_quaternion[3];
            }
        } else if ((R[1] + R[3]) > 0.0f && (R[5] + R[7]) < 0.0f) {
            m_quaternion[3] = -m_quaternion[3];
        }

        // 如果q1应为负，则对向量分量取反
        if ((R[5] - R[7]) < 0.0f) {
            m_quaternion[1] = -m_quaternion[1];
            m_quaternion[2] = -m_quaternion[2];
            m_quaternion[3] = -m_quaternion[3];
        }
    }

    // 最后重新归一化四元数
    fmag = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] +
                             m_quaternion[1] * m_quaternion[1] +
                             m_quaternion[2] * m_quaternion[2] +
                             m_quaternion[3] * m_quaternion[3]);

    m_quaternion[0] *= fmag;
    m_quaternion[1] *= fmag;
    m_quaternion[2] *= fmag;
    m_quaternion[3] *= fmag;

    // 从四元数计算欧拉角
    convertQuaternionToEulerAngle();
}

/**
 * @brief AHRS姿态解算更新
 * @param gyro 陀螺仪数据
 * @param accel 加速度计数据
 * @param magnet 磁力计数据
 * @return const Vector3f& 欧拉角解算结果
 * @note 包含姿态解算完整流程, 数据处理、四元数更新、欧拉角计算
 * @note 为确保数据完整性, 在数据处理过程中禁止FreeRTOS任务切换
 */
const AHRS::Vector3f &AHRS::update(const Vector3f &gyro, const Vector3f &accel, const Vector3f &magnet)
{
    m_gyro   = gyro; // 转存传感器数据
    m_accel  = accel;
    m_magnet = magnet;
    taskENTER_CRITICAL(); // 进入临界区, 禁止任务切换
    dataProcess();
    convertQuaternionToEulerAngle();
    taskEXIT_CRITICAL(); // 退出临界区, 允许任务切换
    return m_eulerAngle;
}

/**
 * @brief 获取四元数
 * @return const fp32* 四元数数组指针
 */
const fp32 *AHRS::getQuaternion() const
{
    return m_quaternion;
}

/**
 * @brief 获取欧拉角
 * @return const Vector3f& 欧拉角
 */
const AHRS::Vector3f &AHRS::getEulerAngle() const
{
    return m_eulerAngle;
}

/**
 * @brief 构造函数
 * @note 初始化成员变量
 */
AHRS::AHRS()
    : m_gyro(), m_accel(), m_magnet(), m_eulerAngle()
{
    m_quaternion[0] = 1.0f; // 初始化四元数为[1, 0, 0, 0]
    m_quaternion[1] = 0.0f;
    m_quaternion[2] = 0.0f;
    m_quaternion[3] = 0.0f;
}

/**
 * @brief 四元数转欧拉角
 */
void AHRS::convertQuaternionToEulerAngle()
{
    // 提取四元数分量
    // 四元数顺序为 [w, x, y, z]
    fp32 q0 = m_quaternion[0];
    fp32 q1 = m_quaternion[1];
    fp32 q2 = m_quaternion[2];
    fp32 q3 = m_quaternion[3];

    // 计算欧拉角 (ZYX顺序, 即yaw-pitch-roll)

    // Roll (x轴旋转) - 绕X轴旋转的角度
    m_eulerAngle.x = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

    // Pitch (y轴旋转) - 绕Y轴旋转的角度
    // 处理万向锁问题，确保结果在[-π/2, π/2]范围内
    fp32 sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        m_eulerAngle.y = copysignf(MATH_PI / 2, sinp); // 使用copysignf确保正确的符号
    else
        m_eulerAngle.y = asinf(sinp);

    // Yaw (z轴旋转) - 绕Z轴旋转的角度
    m_eulerAngle.z = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}
/******************************************************************************
 *                            Mahony类实现
 ******************************************************************************/

/**
 * @brief Mahony姿态解算构造函数
 * @param sampleFreq 采样频率, 默认为0表示使用不定频模式
 * @param accelFilterNum 加速度计低通滤波器系数, 默认为0表示使用大疆官方例程参数
 * @param Kp 比例增益, 默认为0.5
 * @param Ki 积分增益, 默认为0.0
 * @note sampleFreq设置为定频运行时的采样频率(单位Hz), 设置为0表示不定频使用DWT计时, 准确的采样间隔时间有利于更精确的积分计算
 * accelFilterNum为加速度计IIR低通滤波器系数, 默认为0表示使用大疆官方例程参数
 * Kp和Ki分别为比例和积分增益, 用于Mahony算法的PI补偿器
 */
Mahony::Mahony(fp32 sampleFreq, Vector3f accelFilterNum, fp32 Kp, fp32 Ki)
    : AHRS(), m_sampleFreq(sampleFreq), m_deltaTime(0), m_lastUpdateTimestamp(0), m_integralFBx(0), m_integralFBy(0), m_integralFBz(0), m_twoKi(2 * Ki), m_twoKp(2 * Kp), m_accelFilterHistory(), m_accelFiltered()
{
    DWT_Init(); // 初始化DWT计时器, 用于不定频模式下的采样周期计算
    if (accelFilterNum.dot(accelFilterNum)) {
        m_accelFilterNum = accelFilterNum; // 用户自定义滤波器参数
    } else {
        m_accelFilterNum.x = 1.929454039488895f; // 大疆官方例程滤波器参数
        m_accelFilterNum.y = -0.93178349823448126f;
        m_accelFilterNum.z = 0.002329458745586203f;
    }
}

/**
 * @brief 重置Mahony姿态解算相关数据
 * @note 重置积分项、滤波器历史数据, 调用AHRS::reset()重置四元数和欧拉角
 */
void Mahony::reset()
{
    AHRS::reset();
    m_integralFBx   = 0;
    m_integralFBy   = 0;
    m_integralFBz   = 0;
    m_accelFiltered = 0;
    for (int i = 0; i < 3; i++) {
        m_accelFilterHistory[i] = 0;
    }
}

/**
 * @brief Mahony姿态解算数据处理
 * @note 对加速度进行滤波并计算采样周期后调用Mahony姿态解算核心函数
 */
void Mahony::dataProcess()
{
    // 加速度低通滤波
    filterAccel();
    // 计算采样周期
    if (m_sampleFreq) {
        m_deltaTime = 1.0f / m_sampleFreq; // 定频运行模式
    } else {
        m_deltaTime = DWT_GetDeltaTime(&m_lastUpdateTimestamp); // 不定频运行模式
        GSRLMath::constrain(m_deltaTime, 0.1f);   // 限制采样周期在0.1s以内
    }
    // 选择姿态解算算法, 在磁力计数据无效时使用六轴融合算法
    if ((m_magnet.x == 0.0f) && (m_magnet.y == 0.0f) && (m_magnet.z == 0.0f)) {
        Mahony::sixAxisProcess(m_gyro.x, m_gyro.y, m_gyro.z, m_accelFiltered.x, m_accelFiltered.y, m_accelFiltered.z);
    } else {
        Mahony::nineAxisProcess(m_gyro.x, m_gyro.y, m_gyro.z, m_accelFiltered.x, m_accelFiltered.y, m_accelFiltered.z, m_magnet.x, m_magnet.y, m_magnet.z);
    }
}

/**
 * @brief Mahony六轴姿态解算核心函数
 * @param gx 陀螺仪x轴数据
 * @param gy 陀螺仪y轴数据
 * @param gz 陀螺仪z轴数据
 * @param ax 加速度计x轴数据
 * @param ay 加速度计y轴数据
 * @param az 加速度计z轴数据
 * @note 修改自Mahony官方算法https://x-io.co.uk/open-source-imu-and-ahrs-algorithms
 */
void Mahony::sixAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az)
{
    fp32 recipNorm;
    fp32 halfvx, halfvy, halfvz;
    fp32 halfex, halfey, halfez;
    fp32 qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = GSRLMath::invSqrt(ax * ax + ay * ay + az * az); // 该函数返回平方根的倒数
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        // 通过四元数得到理论重力加速度向量g
        // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
        // 这样处理目的是减少乘法运算量
        halfvx = m_quaternion[1] * m_quaternion[3] - m_quaternion[0] * m_quaternion[2];
        halfvy = m_quaternion[0] * m_quaternion[1] + m_quaternion[2] * m_quaternion[3];
        halfvz = m_quaternion[0] * m_quaternion[0] - 0.5f + m_quaternion[3] * m_quaternion[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        // 对实际重力加速度向量v与理论重力加速度向量g做外积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if (m_twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            m_integralFBx += m_twoKi * halfex * m_deltaTime;
            m_integralFBy += m_twoKi * halfey * m_deltaTime;
            m_integralFBz += m_twoKi * halfez * m_deltaTime;

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += m_integralFBx;
            gy += m_integralFBy;
            gz += m_integralFBz;
        } else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            m_integralFBx = 0.0f;
            m_integralFBy = 0.0f;
            m_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += m_twoKp * halfex;
        gy += m_twoKp * halfey;
        gz += m_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * m_deltaTime); // pre-multiply common factors
    gy *= (0.5f * m_deltaTime);
    gz *= (0.5f * m_deltaTime);
    qa = m_quaternion[0];
    qb = m_quaternion[1];
    qc = m_quaternion[2];
    m_quaternion[0] += (-qb * gx - qc * gy - m_quaternion[3] * gz);
    m_quaternion[1] += (qa * gx + qc * gz - m_quaternion[3] * gy);
    m_quaternion[2] += (qa * gy - qb * gz + m_quaternion[3] * gx);
    m_quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] + m_quaternion[1] * m_quaternion[1] + m_quaternion[2] * m_quaternion[2] + m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;
}

/**
 * @brief Mahony九轴姿态解算核心函数
 * @param gx 陀螺仪x轴数据
 * @param gy 陀螺仪y轴数据
 * @param gz 陀螺仪z轴数据
 * @param ax 加速度计x轴数据
 * @param ay 加速度计y轴数据
 * @param az 加速度计z轴数据
 * @param mx 磁力计x轴数据
 * @param my 磁力计y轴数据
 * @param mz 磁力计z轴数据
 * @note 修改自Mahony官方算法https://x-io.co.uk/open-source-imu-and-ahrs-algorithms
 */
void Mahony::nineAxisProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, hz, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = GSRLMath::invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        // 将磁力计得到的实际磁场向量m单位化
        recipNorm = GSRLMath::invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        // 辅助变量，以避免重复运算
        q0q0 = m_quaternion[0] * m_quaternion[0];
        q0q1 = m_quaternion[0] * m_quaternion[1];
        q0q2 = m_quaternion[0] * m_quaternion[2];
        q0q3 = m_quaternion[0] * m_quaternion[3];
        q1q1 = m_quaternion[1] * m_quaternion[1];
        q1q2 = m_quaternion[1] * m_quaternion[2];
        q1q3 = m_quaternion[1] * m_quaternion[3];
        q2q2 = m_quaternion[2] * m_quaternion[2];
        q2q3 = m_quaternion[2] * m_quaternion[3];
        q3q3 = m_quaternion[3] * m_quaternion[3];

        // Reference direction of Earth's magnetic field
        // 通过磁力计测量值与坐标转换矩阵得到大地坐标系下的理论地磁向量
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        hz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        bx = sqrt(hx * hx + hy * hy);
        bz = hz;

        // Estimated direction of gravity and magnetic field
        // 将理论重力加速度向量与理论地磁向量变换至机体坐标系
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        // 通过向量外积得到重力加速度向量和地磁向量的实际值与测量值之间误差
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if (m_twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            m_integralFBx += m_twoKi * halfex * m_deltaTime;
            m_integralFBy += m_twoKi * halfey * m_deltaTime;
            m_integralFBz += m_twoKi * halfez * m_deltaTime;

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += m_integralFBx;
            gy += m_integralFBy;
            gz += m_integralFBz;
        } else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            m_integralFBx = 0.0f;
            m_integralFBy = 0.0f;
            m_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += m_twoKp * halfex;
        gy += m_twoKp * halfey;
        gz += m_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * m_deltaTime); // pre-multiply common factors
    gy *= (0.5f * m_deltaTime);
    gz *= (0.5f * m_deltaTime);
    qa = m_quaternion[0];
    qb = m_quaternion[1];
    qc = m_quaternion[2];
    m_quaternion[0] += (-qb * gx - qc * gy - m_quaternion[3] * gz);
    m_quaternion[1] += (qa * gx + qc * gz - m_quaternion[3] * gy);
    m_quaternion[2] += (qa * gy - qb * gz + m_quaternion[3] * gx);
    m_quaternion[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] + m_quaternion[1] * m_quaternion[1] + m_quaternion[2] * m_quaternion[2] + m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;
}

/**
 * @brief 加速度计数据滤波
 * @note 使用IIR低通滤波器对加速度计数据进行滤波
 */
void Mahony::filterAccel()
{
    // 滚动保存滤波历史数据
    m_accelFilterHistory[0] = m_accelFilterHistory[1];
    m_accelFilterHistory[1] = m_accelFilterHistory[2];
    // x轴滤波
    m_accelFilterHistory[2].x = m_accelFilterHistory[1].x * m_accelFilterNum.x + m_accelFilterHistory[0].x * m_accelFilterNum.y + m_accel.x * m_accelFilterNum.z;
    // y轴滤波
    m_accelFilterHistory[2].y = m_accelFilterHistory[1].y * m_accelFilterNum.x + m_accelFilterHistory[0].y * m_accelFilterNum.y + m_accel.y * m_accelFilterNum.z;
    // z轴滤波
    m_accelFilterHistory[2].z = m_accelFilterHistory[1].z * m_accelFilterNum.x + m_accelFilterHistory[0].z * m_accelFilterNum.y + m_accel.z * m_accelFilterNum.z;
    // 更新滤波后的加速度计数据
    m_accelFiltered = m_accelFilterHistory[2];
}
