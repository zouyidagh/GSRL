# J60 电机驱动模块使用说明以及实例代码

作者：赵辰硕、张楚清
版本：v1.0.0
最后更新日期：2026-01-08

---

## 1：概述

本 J60 电机驱动基于 GSRL 库，适配云深处 J60 关节电机（12:1 减速比），实现了基于 CAN 总线（1Mbps）的高性能控制。

模块基于 C++ 封装，包含：
- 电机驱动类（MotorJ60）
- J60 专用协议解析与打包逻辑（PD控制 + 前馈力矩）

## 2：功能特性

本驱动可实现：
- **混合控制模式**：同时支持位置(P)、速度(V)、力矩(T)前馈以及 Kp、Kd 参数的动态调整。
  - 公式：$T = K_p(P_{des} - P_{fb}) + K_d(V_{des} - V_{fb}) + T_{ff}$
- **状态管理**：
  - 电机使能 (Enable)
  - 电机失能 (Disable)
  - 错误清除 (Clear Error)
  - 零点设置 (Set Zero Position)
- **实时获取电机通过 CAN 返回的运行状态**：
  - 电机角度 (rad)
  - 角速度 (rad/s)
  - 实际力矩 (Nm)
  - 电机温度 (℃)
  - 错误状态 (过压、欠压、过温等)

特性：
- 采用 J60 官方协议 V2.0 (单帧控制)
- 自动处理命令索引（Disable -> Enable -> Control）
- 数据自动限幅（P: ±40rad, V: ±40rad/s, T: ±40Nm）与归一化
- 每次控制循环自动发送心跳包

## 3：适用环境

- MCU：STM32 系列 (F4/H7 等)
- 开发环境：VS Code + EIDE / Keil
- 依赖库：
  - STM32 HAL `drv_can.hpp`
  - 自定义库：`dvc_motor.hpp`
- 通信环境：CAN (波特率 1M)

## 4：典型用法实例

```cpp
#include "dvc_motor.hpp"
#include "main.h"

// 1. 创建 J60 电机实例
// 参数：电机ID (通常为 1), 控制器指针 (J60通常使用内部PD，此处给 nullptr 或空控制器)
MotorJ60 j60Motor(1, nullptr);

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

int main()
{
    // 初始化 CAN，绑定回调 (具体函数名依据你的 HAL 封装)
    CAN_Init(&hcan1, can1RxCallback);

    // 2. 使能电机
    // 注意：J60 上电后需要发送使能命令才能进入控制模式
    // 本驱动会自动在后续的 convert 调用中处理使能命令状态
    j60Motor.enable(); 

    while (1) {
        // 3. 设定控制目标
        // 参数顺序：TargetAngle, TargetVelocity, FeedforwardTorque, Kp, Kd
        // 示例：保持 0 位置，刚度 5.0， 阻尼 0.1
        j60Motor.setControlParams(0.0f, 0.0f, 0.0f, 5.0f, 0.1f);
        
        // 或者使用特定的闭环函数 (如果基类方法已适配 J60)
        // j60Motor.angleClosedloopControl(0.0f); 
        
        // 4. 计算并打包 CAN 数据
        j60Motor.convertControllerOutputToMotorControlData();

        // 5. 发送数据
        transmitMotorsControlData();

        // 简单的延时，控制频率建议 500Hz - 1kHz
        HAL_Delay(1); 
    }
}

// CAN 接收中断回调
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    // 6. 解析反馈数据
    j60Motor.decodeCanRxMessageFromISR(pRxMsg);
}

/**
* @brief 传输电机控制数据
*/
inline void transmitMotorsControlData()
{
    // 获取电机 CAN 消息头和数据
    CAN_TxHeaderTypeDef *txHeader = (CAN_TxHeaderTypeDef *)j60Motor.getMotorControlHeader();
    uint8_t *txData = (uint8_t *)j60Motor.getMotorControlData();
    
    // 发送 (伪代码，请使用你的 HAL 库函数)
    // can_send_message(&hcan1, txHeader, txData);
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1, txHeader, txData, &mailbox);
}
```

---

## 5: 注意事项

1. **ID 设置**：J60 电机的 ID 默认通常为 1。请确保代码中的 ID 与实际电机 DIP 开关或软件设置一致。
2. **上电顺序**：建议先给控制板上电，再给电机上电。电机上电后默认为失能状态（阻尼极小）。
3. **参数安全**：
   - `Kp` 和 `Kd` 不要设置过大，否则会引起剧烈震荡。建议从小到大调试（如 Kp=1.0, Kd=0.1 开始）。
   - `TargetAngle` (P) 范围为 -40 ~ +40 rad，超出范围会被协议限幅。
4. **散热**：J60 功率密度较高，高负载运行时请注意监测 `getTemperature()` 返回值。
