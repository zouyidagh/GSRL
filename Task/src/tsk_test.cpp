/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_motor.hpp"
#include "dvc_remotecontrol.hpp"
#include "drv_spi.h"
#include "dvc_imu.hpp"

/* Define --------------------------------------------------------------------*/
// PID
SimplePID::PIDParam param = {
    10.0f,  // Kp
    0.0f,  // Ki
    500.0f,   // Kd
    10.0f,  // outputLimit
    0.0f    // intergralLimit
};
SimplePID myPID(SimplePID::PID_POSITION, param);
// Motor
MotorDM4310 motor(1, 0, 3.1415926f, 40, 15, &myPID);
// RemoteControl
Dr16RemoteControl dr16;
// IMU
using Vector3f = GSRLMath::Vector3f;
Mahony ahrs{};
BMI088::CalibrationInfo cali = {
    {0.0f, 0.0f, 0.0f}, // gyroOffset
    {0.0f, 0.0f, 0.0f}, // accelOffset
    {0.0f, 0.0f, 0.0f},  // magnetOffset
    {GSRLMath::Matrix33f::MatrixType::IDENTITY} // installSpinMatrix
};
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);
GSRLMath::Vector3f eulerAngle;

/* Variables -----------------------------------------------------------------*/
can_rx_message_t rxMessage[16]; // CAN接收消息缓冲区

/* Function prototypes -------------------------------------------------------*/
extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length);
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

/* User code -----------------------------------------------------------------*/

uint8_t txData, rxData;
/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback); // 初始化CAN1
    UART_Init(&huart3, dr16ITCallback, 36); // 初始化DR16串口
    uint16_t count = 0;
    fp32 angle = 0.0f;
    while (imu.init() == false)
    {
        osDelay(100);
    } // 初始化IMU
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    while(1)
    {
        eulerAngle = imu.solveAttitude(); // 解算姿态
        eulerAngle = eulerAngle * 57.3f; // 弧度转角度
        count ++;
        if (count>1000)
        {
            angle = GSRLMath::normalizeAngle(angle + MATH_PI * 2 / 3);
            count = 0;
        }
        motor.angleClosedloopControl(angle);

        transmitMotorsControlData();
        vTaskDelayUntil(&taskLastWakeTime, 1); // 确保任务以定周期1ms运行
    }
}

/**
 * @brief DR16接收中断回调函数
 * @param Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length)
{
    dr16.receiveDr16RxDataFromISR(Buffer);
}


extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    motor.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief 发送电机控制数据
 */
inline void transmitMotorsControlData()
{
    const uint8_t* data = motor.getMotorControlData();
    uint32_t send_mail_box;
    HAL_CAN_AddTxMessage(&hcan1, motor.getMotorControlHeader(), data, &send_mail_box);
}
