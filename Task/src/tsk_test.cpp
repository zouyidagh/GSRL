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
// #include "dvc_referee.hpp"
#include "dvc_remotecontrol.hpp"
#include "drv_spi.h"
#include "dvc_imu.hpp"
#include "main.h"

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
// MotorDM4310 motor(1, 0, 3.1415926f, 40, 15, &myPID);
MotorJ60 motor(1, &myPID); // 使用电机ID=1，对应控制ID 0x81、反馈ID 0x91
// RemoteControl
Dr16RemoteControl dr16;
// Referee
// RefereeParser referee(RefereeLink::Main);

// Test Counters
volatile uint32_t referee_update_count = 0;
volatile uint32_t isr_count = 0;
volatile uint32_t sof_count = 0;

// CAN/J60 调试探针
volatile uint32_t can_tx_ok = 0;
volatile uint32_t can_tx_err = 0;
volatile uint32_t can_rx_hit = 0;
volatile uint32_t can_tx_last_id = 0;
volatile uint8_t  can_tx_last_dlc = 0;
volatile uint32_t can_last_error = 0;   // HAL_CAN_GetError
volatile uint32_t j60_enable_left = 0;   // 剩余使能帧计数
volatile uint32_t j60_state = 0;         // MotorJ60::MotorState
volatile uint32_t j60_fb_id = 0;         // 最近收到的反馈ID
volatile uint8_t  j60_fb_raw_err = 0;    // 反馈错误码
volatile uint32_t loop_count = 0;        // 循环计数

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
extern "C" void dr16ITCallback(uint8_t *Buffer, uint16_t Length);
extern "C" void refereeITCallback(uint8_t *Buffer, uint16_t Length);
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
    UART_Init(&huart6, dr16ITCallback, 36);            // 初始化DR16串口
    // UART_Init(&huart6, refereeITCallback, 128);        // 初始化裁判系统串口
    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    
    // J60 电机初始化
    motor.enable(); // 使能电机
    transmitMotorsControlData();
    motor.constructRTR(2);
    transmitMotorsControlData();
    // motor.setControlParams(0.0f, 0.0f, 0.0f, 10.0f, 1.0f); // 设置初始参数: 位置模式, P=0, Kp=10, Kd=1

    float time = 0.0f;
    while (1) {
        // 生成正弦波位置指令: 幅度 1.0 rad, 频率 0.5 Hz
        float target_pos = 1.0f * sinf(2.0f * 3.14159f * 0.5f * time);
        // J60协议: Kp=[0,1023], Kd=[0.5,1.0]
        motor.setControlParams(target_pos, 0.0f, 0.0f, 100.0f, 0.8f);
        motor.convertControllerOutputToMotorControlData(); // 打包控制数据
        transmitMotorsControlData();

        loop_count++;
        time += 0.001f; // 1ms 步长
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
