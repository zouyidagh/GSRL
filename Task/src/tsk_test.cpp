/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
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
#include "dvc_remotecontrol.hpp"
#include "drv_spi.h"
#include "dvc_imu.hpp"

/* Define --------------------------------------------------------------------*/
// PID
SimplePID::PIDParam param = {
    10.0f,  // Kp
    0.0f,   // Ki
    500.0f, // Kd
    10.0f,  // outputLimit
    0.0f    // intergralLimit
};
SimplePID myPID(SimplePID::PID_POSITION, param);
// Motor
MotorDM4310 motor(1, 0, 3.1415926f, 40, 15, &myPID);
// RemoteControl
Dr16RemoteControl dr16;

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length);
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

/* User code -----------------------------------------------------------------*/

/**
 * @brief 测试任务
 * @param argument 任务参数
 */
extern "C" void test_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);                  // 初始化CAN1
    UART_Init(&huart3, dr16ITCallback, 36);            // 初始化DR16串口
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    while (1) {
        motor.openloopControl(0.0f);
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
    dr16.receiveRxDataFromISR(Buffer);
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
    const uint8_t *data = motor.getMotorControlData();
    uint32_t send_mail_box;
    HAL_CAN_AddTxMessage(&hcan1, motor.getMotorControlHeader(), data, &send_mail_box);
}
