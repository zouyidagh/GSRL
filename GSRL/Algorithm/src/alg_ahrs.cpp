/**
 ******************************************************************************
 * @file           : alg_ahrs.cpp
 * @brief          : AHRS姿态解算算法
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "drv_misc.h"
#include "alg_ahrs.hpp"
#include "alg_filter.hpp"

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
    m_gyro                  = 0;
    m_accel                 = 0;
    m_magnet                = 0;
    m_eulerAngle            = 0;
    m_motionAccelBodyFrame  = 0;
    m_motionAccelEarthFrame = 0;
    m_quaternion[0]         = 1.0f;
    m_quaternion[1]         = 0.0f;
    m_quaternion[2]         = 0.0f;
    m_quaternion[3]         = 0.0f;
    m_isAhrsInited          = false;
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
    if (!m_isAhrsInited) {
        initQuaternion(); // 根据传感器数据初始化四元数
        init();           // 子类特定初始化(纯虚函数)
        m_isAhrsInited = true;
    }
    taskENTER_CRITICAL(); // 进入临界区, 禁止任务切换
    dataProcess();
    calculateMotionAccel();
    convertQuaternionToEulerAngle();
    taskEXIT_CRITICAL(); // 退出临界区, 允许任务切换
    return m_eulerAngle;
}

/**
 * @brief 获取陀螺仪数据
 * @return const Vector3f& 陀螺仪数据
 */
const AHRS::Vector3f &AHRS::getGyro() const
{
    return m_gyro;
}

/**
 * @brief 获取加速度计数据
 * @return const Vector3f& 加速度计数据
 */
const AHRS::Vector3f &AHRS::getAccel() const
{
    return m_accel;
}

/**
 * @brief 获取机体坐标系下的运动加速度
 * @return const Vector3f& 机体坐标系下的运动加速度
 */
const AHRS::Vector3f &AHRS::getMotionAccelBodyFrame() const
{
    return m_motionAccelBodyFrame;
}

/**
 * @brief 获取大地坐标系下的运动加速度
 * @return const Vector3f& 大地坐标系下的运动加速度
 */
const AHRS::Vector3f &AHRS::getMotionAccelEarthFrame() const
{
    return m_motionAccelEarthFrame;
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
    : m_isAhrsInited(false)
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
    if (fabsf(sinp) >= 1) {
        m_eulerAngle.y = copysignf(MATH_PI / 2, sinp); // 使用copysignf确保正确的符号
    } else {
        m_eulerAngle.y = asinf(sinp);
    }

    // Yaw (z轴旋转) - 绕Z轴旋转的角度
    m_eulerAngle.z = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

/**
 * @brief 根据传感器数据初始化四元数
 * @note 有磁力计时使用TRIAD方法(加速度+磁力计)构建完整旋转矩阵,
 *       无磁力计时仅用加速度对齐重力方向(yaw设为0)
 * @note 在首次update()中、子类init()之前调用, 此时m_accel和m_magnet已就绪
 */
void AHRS::initQuaternion()
{
    // 归一化加速度计数据, 得到重力方向单位向量
    fp32 recipNorm = GSRLMath::invSqrt(m_accel.x * m_accel.x + m_accel.y * m_accel.y + m_accel.z * m_accel.z);
    fp32 ax        = m_accel.x * recipNorm;
    fp32 ay        = m_accel.y * recipNorm;
    fp32 az        = m_accel.z * recipNorm;

    bool hasMag = m_magnet.x != 0.0f || m_magnet.y != 0.0f || m_magnet.z != 0.0f;

    if (hasMag) {
        // 九轴初始化: TRIAD方法 + Shepperd四元数提取
        recipNorm = GSRLMath::invSqrt(m_magnet.x * m_magnet.x + m_magnet.y * m_magnet.y + m_magnet.z * m_magnet.z);
        fp32 mx   = m_magnet.x * recipNorm;
        fp32 my   = m_magnet.y * recipNorm;
        fp32 mz   = m_magnet.z * recipNorm;

        // East = mag × accel
        fp32 ex   = my * az - mz * ay;
        fp32 ey   = mz * ax - mx * az;
        fp32 ez   = mx * ay - my * ax;
        recipNorm = GSRLMath::invSqrt(ex * ex + ey * ey + ez * ez);
        ex *= recipNorm;
        ey *= recipNorm;
        ez *= recipNorm;

        // North = accel × East
        fp32 nx = ay * ez - az * ey;
        fp32 ny = az * ex - ax * ez;
        fp32 nz = ax * ey - ay * ex;

        // 旋转矩阵 R (NED→Body, 行优先)
        fp32 R[9] = {nx, ny, nz, ex, ey, ez, ax, ay, az};

        // Shepperd方法: 旋转矩阵→四元数
        fp32 trace = R[0] + R[4] + R[8];
        if (trace > 0.0f) {
            fp32 s          = sqrtf(trace + 1.0f);
            m_quaternion[0] = s * 0.5f;
            s               = 0.5f / s;
            m_quaternion[1] = (R[7] - R[5]) * s;
            m_quaternion[2] = (R[2] - R[6]) * s;
            m_quaternion[3] = (R[3] - R[1]) * s;
        } else if ((R[0] >= R[4]) && (R[0] >= R[8])) {
            fp32 s          = sqrtf(1.0f + R[0] - R[4] - R[8]);
            m_quaternion[1] = s * 0.5f;
            s               = 0.5f / s;
            m_quaternion[0] = (R[7] - R[5]) * s;
            m_quaternion[2] = (R[1] + R[3]) * s;
            m_quaternion[3] = (R[2] + R[6]) * s;
        } else if (R[4] > R[8]) {
            fp32 s          = sqrtf(1.0f + R[4] - R[0] - R[8]);
            m_quaternion[2] = s * 0.5f;
            s               = 0.5f / s;
            m_quaternion[0] = (R[2] - R[6]) * s;
            m_quaternion[1] = (R[1] + R[3]) * s;
            m_quaternion[3] = (R[5] + R[7]) * s;
        } else {
            fp32 s          = sqrtf(1.0f + R[8] - R[0] - R[4]);
            m_quaternion[3] = s * 0.5f;
            s               = 0.5f / s;
            m_quaternion[0] = (R[3] - R[1]) * s;
            m_quaternion[1] = (R[2] + R[6]) * s;
            m_quaternion[2] = (R[5] + R[7]) * s;
        }
    } else {
        // 六轴初始化: 仅用加速度对齐重力方向, yaw = 0
        fp32 roll  = atan2f(ay, az);
        fp32 pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

        fp32 cr = cosf(roll * 0.5f);
        fp32 sr = sinf(roll * 0.5f);
        fp32 cp = cosf(pitch * 0.5f);
        fp32 sp = sinf(pitch * 0.5f);
        // yaw = 0 → cy = 1, sy = 0, ZYX欧拉角转四元数
        m_quaternion[0] = cr * cp;
        m_quaternion[1] = sr * cp;
        m_quaternion[2] = cr * sp;
        m_quaternion[3] = -sr * sp;
    }

    // 归一化四元数
    recipNorm = GSRLMath::invSqrt(m_quaternion[0] * m_quaternion[0] +
                                  m_quaternion[1] * m_quaternion[1] +
                                  m_quaternion[2] * m_quaternion[2] +
                                  m_quaternion[3] * m_quaternion[3]);
    m_quaternion[0] *= recipNorm;
    m_quaternion[1] *= recipNorm;
    m_quaternion[2] *= recipNorm;
    m_quaternion[3] *= recipNorm;
}

/**
 * @brief 计算运动加速度
 * @note 从m_accel中减去重力加速度分量, 得到机体坐标系下的运动加速度,
 *       然后通过四元数旋转矩阵将其转换到大地坐标系
 * @note 在dataProcess()执行完成后调用, 此时四元数已更新
 */
void AHRS::calculateMotionAccel()
{
    fp32 q0 = m_quaternion[0];
    fp32 q1 = m_quaternion[1];
    fp32 q2 = m_quaternion[2];
    fp32 q3 = m_quaternion[3];

    // 计算重力加速度在机体坐标系下的分量(单位向量)
    fp32 gx = 2.0f * (q1 * q3 - q0 * q2);
    fp32 gy = 2.0f * (q0 * q1 + q2 * q3);
    fp32 gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 从加速度计数据中减去重力分量, 得到机体坐标系下的运动加速度
    constexpr fp32 GRAVITY   = 9.8f;
    m_motionAccelBodyFrame.x = m_accel.x - gx * GRAVITY;
    m_motionAccelBodyFrame.y = m_accel.y - gy * GRAVITY;
    m_motionAccelBodyFrame.z = m_accel.z - gz * GRAVITY;

    // 将机体坐标系下的运动加速度转换到大地坐标系
    m_motionAccelEarthFrame.x = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * m_motionAccelBodyFrame.x +
                                2.0f * (q1 * q2 - q0 * q3) * m_motionAccelBodyFrame.y +
                                2.0f * (q1 * q3 + q0 * q2) * m_motionAccelBodyFrame.z;

    m_motionAccelEarthFrame.y = 2.0f * (q1 * q2 + q0 * q3) * m_motionAccelBodyFrame.x +
                                (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * m_motionAccelBodyFrame.y +
                                2.0f * (q2 * q3 - q0 * q1) * m_motionAccelBodyFrame.z;

    m_motionAccelEarthFrame.z = 2.0f * (q1 * q3 - q0 * q2) * m_motionAccelBodyFrame.x +
                                2.0f * (q2 * q3 + q0 * q1) * m_motionAccelBodyFrame.y +
                                (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * m_motionAccelBodyFrame.z;
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
Mahony::Mahony(fp32 sampleFreq, Vector3f accelFilterNum, fp32 kp, fp32 ki)
    : m_sampleFreq(sampleFreq), m_deltaTime(0), m_lastUpdateTimestamp(0), m_integralFBx(0), m_integralFBy(0), m_integralFBz(0), m_twoKi(2 * ki), m_twoKp(2 * kp), m_accelFilterHistory()
{
    if (accelFilterNum.x != 0.0f || accelFilterNum.y != 0.0f || accelFilterNum.z != 0.0f) {
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
    for (auto &i : m_accelFilterHistory) {
        i = 0;
    }
}

/**
 * @brief 初始化Mahony姿态解算
 */
void Mahony::init()
{
    // 四元数已由基类initQuaternion()初始化

    DWT_Init(); // 初始化DWT计时器, 用于不定频模式下的采样周期计算

    // 初始化DWT计数器时间戳
    if (m_sampleFreq == 0.0f && DWT_IsInit()) {
        m_lastUpdateTimestamp = DWT_GetTimestamp();
    }
}

/**
 * @brief 获取滤波后的加速度计数据
 * @return const Vector3f& 滤波后的加速度计数据
 */
const Mahony::Vector3f &Mahony::getAccel() const
{
    return m_accelFiltered;
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
    if (m_sampleFreq > 0.0f) {
        m_deltaTime = 1.0f / m_sampleFreq; // 定频运行模式
    } else {
        m_deltaTime = DWT_GetDeltaTime(&m_lastUpdateTimestamp); // 不定频运行模式
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
    if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f)) {

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
    if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f)) {

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

/******************************************************************************
 *                             QuaternionEKF类实现
 ******************************************************************************/

/**
 * @brief QuaternionEKF构造函数
 * @param sampleFreq 给定刷新频率(Hz), 给0则使用DWT自动识别
 * @param quatProcessNoise 四元数过程噪声基准(典型值10)
 * @param biasProcessNoise 陀螺仪零偏过程噪声基准(典型值0.001)
 * @param measNoise 加速度计量测噪声基准(典型值1e6)
 * @param lambda 衰减系数(0~1], 防止零偏协方差过度收敛, 1为不衰减(典型值0.9996)
 * @param accLpfCoef 加速度计一阶低通滤波时间常数(s), 0表示不滤波
 * @param isCheckChiSquare 是否启用卡方检验
 * @param chiSquareThreshold 卡方检验阈值, 卡方值大于此阈值时忽略加速度观测(典型值1e-8)
 * @note 仅保存参数, KF矩阵的初始化在init()中完成（首次update时由基类触发）
 */
QuaternionEKF::QuaternionEKF(fp32 sampleFreq,
                             fp32 quatProcessNoise,
                             fp32 biasProcessNoise,
                             fp32 measNoise,
                             fp32 lambda,
                             fp32 accLpfCoef,
                             bool isCheckChiSquare,
                             fp32 chiSquareThreshold)
    : m_sampleFreq(sampleFreq),
      m_lastUpdateTimestamp(0),
      m_deltaTime(0.0f),
      m_quatProcessNoise(quatProcessNoise),
      m_biasProcessNoise(biasProcessNoise),
      m_measNoise(measNoise),
      m_lambda(lambda > 1.0f ? 1.0f : lambda),
      m_chiSquareThreshold(chiSquareThreshold),
      m_accLpfCoef(accLpfCoef),
      m_isCheckChiSquare(isCheckChiSquare),
      m_convergeFlag(false),
      m_stableFlag(false),
      m_errorCount(0),
      m_adaptiveGainScale(1.0f),
      m_gyroNorm(0.0f),
      m_accelNorm(0.0f) {}

/**
 * @brief 重置QuaternionEKF姿态解算相关数据
 */
void QuaternionEKF::reset()
{
    AHRS::reset();
    m_kalmanFilter.reset();
    m_accelFiltered     = 0;
    m_gyroBias          = 0;
    m_orientationCosine = 0;
    m_convergeFlag      = false;
    m_stableFlag        = false;
    m_errorCount        = 0;
    m_adaptiveGainScale = 1.0f;
    m_gyroNorm          = 0.0f;
    m_accelNorm         = 0.0f;
}
/**
 * @brief 初始化EKF姿态解算
 * @note 在第一次update被调用时由基类触发执行，完成DWT、KF矩阵初始化
 */
void QuaternionEKF::init()
{
    DWT_Init();

    if (m_sampleFreq == 0.0f && DWT_IsInit()) {
        m_lastUpdateTimestamp = DWT_GetTimestamp();
    }

    using M = KF;

    m_kalmanFilter.setTransitionMatrix(M::StateMatrix::Identity());
    m_kalmanFilter.setObservationMatrix(M::ObsMatrix::Zero());
    m_kalmanFilter.setProcessNoise(M::StateMatrix::Zero());
    m_kalmanFilter.setMeasurementNoise(M::MeasMatrix::Zero());

    M::StateMatrix P = M::StateMatrix::Constant(0.1f);
    P.diagonal().setConstant(1.0f);
    m_kalmanFilter.setCovariance(P);

    // 将基类initQuaternion()已初始化的四元数写入EKF状态向量
    M::StateVector x0;
    x0 << m_quaternion[0], m_quaternion[1], m_quaternion[2], m_quaternion[3], 0.f, 0.f;
    m_kalmanFilter.setState(x0);

    // 初始化低通滤波器状态
    m_accelFiltered = m_accel;
}

/**
 * @brief QuaternionEKF姿态解算数据处理
 */
void QuaternionEKF::dataProcess()
{
    if (m_sampleFreq > 0.0f) {
        m_deltaTime = 1.0f / m_sampleFreq;
    } else {
        m_deltaTime = DWT_GetDeltaTime(&m_lastUpdateTimestamp);
    }

    ekfProcess(m_gyro.x, m_gyro.y, m_gyro.z, m_accel.x, m_accel.y, m_accel.z);
}

/**
 * @brief EKF姿态解算核心函数
 * @param gx 陀螺仪x轴数据(rad/s)
 * @param gy 陀螺仪y轴数据(rad/s)
 * @param gz 陀螺仪z轴数据(rad/s)
 * @param ax 加速度计x轴数据(m/s²)
 * @param ay 加速度计y轴数据(m/s²)
 * @param az 加速度计z轴数据(m/s²)
 */
void QuaternionEKF::ekfProcess(fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az)
{
    using M       = KF;
    const fp32 dt = m_deltaTime;

    // 减去零偏得到校正后角速度
    fp32 wx = gx - m_gyroBias.x;
    fp32 wy = gy - m_gyroBias.y;
    fp32 wz = gz - m_gyroBias.z;

    fp32 halfwxdt = 0.5f * wx * dt;
    fp32 halfwydt = 0.5f * wy * dt;
    fp32 halfwzdt = 0.5f * wz * dt;

    // 构建状态转移矩阵F
    M::StateMatrix F = M::StateMatrix::Identity();
    F(0, 1)          = -halfwxdt;
    F(0, 2)          = -halfwydt;
    F(0, 3)          = -halfwzdt;
    F(1, 0)          = halfwxdt;
    F(1, 2)          = halfwzdt;
    F(1, 3)          = -halfwydt;
    F(2, 0)          = halfwydt;
    F(2, 1)          = -halfwzdt;
    F(2, 3)          = halfwxdt;
    F(3, 0)          = halfwzdt;
    F(3, 1)          = halfwydt;
    F(3, 2)          = -halfwxdt;
    m_kalmanFilter.setTransitionMatrix(F);

    // 加速度一阶低通滤波
    fp32 lpfDenom = dt + m_accLpfCoef;
    if (lpfDenom > 1e-12f) {
        m_accelFiltered.x = m_accelFiltered.x * m_accLpfCoef / lpfDenom + ax * dt / lpfDenom;
        m_accelFiltered.y = m_accelFiltered.y * m_accLpfCoef / lpfDenom + ay * dt / lpfDenom;
        m_accelFiltered.z = m_accelFiltered.z * m_accLpfCoef / lpfDenom + az * dt / lpfDenom;
    }

    // 归一化加速度作为量测
    fp32 accelInvNorm = GSRLMath::invSqrt(
        m_accelFiltered.x * m_accelFiltered.x +
        m_accelFiltered.y * m_accelFiltered.y +
        m_accelFiltered.z * m_accelFiltered.z);

    M::MeasVector measurement;
    measurement << m_accelFiltered.x * accelInvNorm,
        m_accelFiltered.y * accelInvNorm,
        m_accelFiltered.z * accelInvNorm;

    // 计算载体运动状态(用于发散保护)
    m_gyroNorm  = 1.0f / GSRLMath::invSqrt(wx * wx + wy * wy + wz * wz);
    m_accelNorm = 1.0f / accelInvNorm;

    if (m_gyroNorm < 0.3f && m_accelNorm > 9.3f && m_accelNorm < 10.3f) {
        m_stableFlag = true;
    } else {
        m_stableFlag = false;
    }

    // 动态设置过程噪声Q和量测噪声R
    M::StateMatrix Qm = M::StateMatrix::Zero();
    Qm(0, 0)          = m_quatProcessNoise * dt;
    Qm(1, 1)          = m_quatProcessNoise * dt;
    Qm(2, 2)          = m_quatProcessNoise * dt;
    Qm(3, 3)          = m_quatProcessNoise * dt;
    Qm(4, 4)          = m_biasProcessNoise * dt;
    Qm(5, 5)          = m_biasProcessNoise * dt;
    m_kalmanFilter.setProcessNoise(Qm);

    M::MeasMatrix Rm = M::MeasMatrix::Zero();
    Rm(0, 0)         = m_measNoise;
    Rm(1, 1)         = m_measNoise;
    Rm(2, 2)         = m_measNoise;
    m_kalmanFilter.setMeasurementNoise(Rm);

    // 预测步骤
    m_kalmanFilter.predict();

    // 预测态四元数归一化
    M::StateVector xpred = m_kalmanFilter.getPredictedState();
    fp32 q0 = xpred(0), q1 = xpred(1), q2 = xpred(2), q3 = xpred(3);

    fp32 qInvNorm = GSRLMath::invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= qInvNorm;
    q1 *= qInvNorm;
    q2 *= qInvNorm;
    q3 *= qInvNorm;
    xpred(0) = q0;
    xpred(1) = q1;
    xpred(2) = q2;
    xpred(3) = q3;
    m_kalmanFilter.setState(xpred);

    // Bias协方差fading处理
    M::StateMatrix Ppred = m_kalmanFilter.getPredictedCovariance();
    Ppred(4, 4) /= m_lambda;
    Ppred(5, 5) /= m_lambda;
    if (Ppred(4, 4) > 10000.0f) Ppred(4, 4) = 10000.0f;
    if (Ppred(5, 5) > 10000.0f) Ppred(5, 5) = 10000.0f;

    // 构建观测矩阵H
    fp32 dq0 = 2.0f * q0, dq1 = 2.0f * q1, dq2 = 2.0f * q2, dq3 = 2.0f * q3;

    M::ObsMatrix H = M::ObsMatrix::Zero();
    H(0, 0)        = -dq2;
    H(0, 1)        = dq3;
    H(0, 2)        = -dq0;
    H(0, 3)        = dq1;
    H(1, 0)        = dq1;
    H(1, 1)        = dq0;
    H(1, 2)        = dq3;
    H(1, 3)        = dq2;
    H(2, 0)        = dq0;
    H(2, 1)        = -dq1;
    H(2, 2)        = -dq2;
    H(2, 3)        = dq3;

    // 卡方检验与自适应量测更新
    M::MeasMatrix S    = H * Ppred * H.transpose() + Rm;
    M::MeasMatrix Sinv = S.inverse();

    M::MeasVector hx;
    hx << 2.0f * (q1 * q3 - q0 * q2),
        2.0f * (q0 * q1 + q2 * q3),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    for (int i = 0; i < 3; i++) {
        m_orientationCosine[i] = acosf(fabsf(hx(i)));
    }

    M::MeasVector innovation = measurement - hx;
    fp32 chiSquare           = (innovation.transpose() * Sinv * innovation)(0, 0);

    bool skipCorrection = false;

    if (m_isCheckChiSquare) {
        if (chiSquare < 0.5f * m_chiSquareThreshold) {
            m_convergeFlag = true;
        }

        if (chiSquare > m_chiSquareThreshold && m_convergeFlag) {
            if (m_stableFlag) {
                m_errorCount++;
            } else {
                m_errorCount = 0;
            }

            if (m_errorCount > 50) {
                m_convergeFlag = false;
            } else {
                skipCorrection = true;
            }
        } else {
            if (chiSquare > 0.1f * m_chiSquareThreshold && m_convergeFlag) {
                m_adaptiveGainScale = (m_chiSquareThreshold - chiSquare) / (0.9f * m_chiSquareThreshold);
            } else {
                m_adaptiveGainScale = 1.0f;
            }
            m_errorCount = 0;
        }
    } else {
        m_adaptiveGainScale = 1.0f;
    }

    if (skipCorrection) {
        m_kalmanFilter.setState(xpred);
        m_kalmanFilter.setCovariance(Ppred);
    } else {
        M::GainMatrix K = Ppred * H.transpose() * Sinv;
        K *= m_adaptiveGainScale;

        constexpr fp32 HALF_PI = MATH_PI / 2.0f;
        for (int j = 0; j < 3; j++) {
            K(4, j) *= m_orientationCosine[0] / HALF_PI;
            K(5, j) *= m_orientationCosine[1] / HALF_PI;
        }

        M::StateVector delta = K * innovation;

        if (m_convergeFlag) {
            fp32 biasLimit = 1e-2f * dt;
            GSRLMath::constrain(delta(4), biasLimit);
            GSRLMath::constrain(delta(5), biasLimit);
        }

        delta(3) = 0.0f;

        m_kalmanFilter.setState(xpred + delta);

        M::StateMatrix IKH = M::StateMatrix::Identity() - K * H;
        m_kalmanFilter.setCovariance(IKH * Ppred);
    }

    // 回写状态到AHRS输出
    M::StateVector xhat = m_kalmanFilter.getState();

    q0      = xhat(0);
    q1      = xhat(1);
    q2      = xhat(2);
    q3      = xhat(3);
    fp32 bx = xhat(4), by = xhat(5);

    qInvNorm = GSRLMath::invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= qInvNorm;
    q1 *= qInvNorm;
    q2 *= qInvNorm;
    q3 *= qInvNorm;

    m_gyroBias.x = bx;
    m_gyroBias.y = by;
    m_gyroBias.z = 0.0f;

    m_quaternion[0] = q0;
    m_quaternion[1] = q1;
    m_quaternion[2] = q2;
    m_quaternion[3] = q3;

    xhat(0) = q0;
    xhat(1) = q1;
    xhat(2) = q2;
    xhat(3) = q3;
    m_kalmanFilter.setState(xhat);
}
