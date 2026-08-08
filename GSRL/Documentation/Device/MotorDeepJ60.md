# 云深处 J60 关节电机驱动使用说明

版本：v1.0.0
最后更新日期：2026-04-11

---

## 1：概述

本驱动基于 GSRL 库，为云深处 DeepRobotics J60 关节电机提供 CAN 通信（1Mbps）支持。

模块基于 C++ 封装，包含：

- 电机驱动类 `MotorDeepJ60`
- J60 私有协议（命令索引式 CAN ID）的打包/解析逻辑
- 上电后 DISABLE → ENABLE → CONTROL 的自动状态机，掉线自动重连

## 2：功能特性

本驱动可实现：

- 与 GSRL 框架其他电机一致的基类接口：
  - `torqueCurrentClosedloopControl` 力矩前馈控制（板外 PID）
  - `angleClosedloopControl` / `revolutionsClosedloopControl` 位置闭环
  - `externalClosedloopControl` 外部反馈闭环
- `hardwareMitControl(p, v, t, kp, kd)` 可选的板载 PD 模式（关节板内部执行 PD 控制律）
- `enable()` / `disable()` / `clearError()` 的高级状态控制
- 实时获取电机反馈：
  - 当前角度 `getCurrentAngle()`（单位 rad，范围 [0, 2π)）
  - 多圈绝对位置 `getCurrentRevolutions()`（单位 圈，J60 原生支持 ±6.37 圈）
  - 当前角速度 `getCurrentAngularVelocity()`（单位 rad/s）
  - 当前扭矩 `getCurrentTorqueNm()`（单位 Nm，高精度）
  - 电机温度 / MOSFET 温度（℃）
  - 电机连接状态 `isMotorConected()`

特性：

- 使用 J60 CAN 协议 V2.0（MIT 单帧控制模式）
- 数据自动限幅（P: ±40 rad，V: ±40 rad/s，T: ±40 Nm，Kp: [0,1023]，Kd: [0,51]）
- 掉线后自动重发 ENABLE 命令重连
- 不依赖独立的保活任务，每次 `convert...()` 调用即根据状态机自动选择要发送的帧

## 3：J60 电机准备工作

使用前请调试J60参数：

1. 使用串口调试助手设置关节 ID（默认为 1 但仍需设置）
2. 标定零位（运行时 CAN 协议不提供设零位命令）
3. 合理设置 CAN 超时时间（建议 200ms 左右），避免掉线后电机失控乱跳
4. 若需要使用CAN配置电机参数，请发送 `CMD_SAVE_CONFIG` 永久生效

## 4：典型用法实例

```cpp
#include "dvc_motor.hpp"
#include "alg_pid.hpp"
#include "main.h"

// 1. 构造板外 PID 控制器（力矩输出，单位 Nm）
SimplePID::PIDParam torqueParam = {
    2.0f,    // Kp
    0.05f,   // Ki
    0.0f,    // Kd
    40.0f,   // outputLimit (Nm)
    20.0f    // integralLimit
};
SimplePID j60TorquePID(SimplePID::PID_POSITION, torqueParam);

// 2. 构造 J60 电机实例（关节 ID = 1）
MotorJ60 j60Motor(1, &j60TorquePID);

extern "C" void can1RxCallback(can_rx_message_t *pRxMsg);
inline void transmitMotorsControlData();

int main()
{
    // CAN 初始化（绑定回调，具体函数名依据本项目 BSP 封装）
    CAN_Init(&hcan1, can1RxCallback);

    // 构造即默认意图为使能，无需手动调用 enable()
    // （若希望延迟到某一时刻才使能，可在构造后先调 j60Motor.disable()）

    while (1) {
        // 3. 计算并打包 CAN 控制数据
        //    方式 A：力矩目标闭环（上位控制器算出力矩，Nm -> mNm）
        //    j60Motor.torqueCurrentClosedloopControl(500);   // 目标 0.5 Nm

        //    方式 B：角度目标闭环（基类已自动算出扭矩）
        j60Motor.angleClosedloopControl(0.0f);

        //    方式 C：板载 PD 模式（绕开基类控制器，直接下发 MIT 五元组）
        //    j60Motor.hardwareMitControl(0.0f, 0.0f, 0.0f, 5.0f, 0.1f);

        // 4. 发送 CAN 帧
        transmitMotorsControlData();

        // 控制周期建议 500Hz ~ 1kHz；本示例使用 FreeRTOS 延时
        osDelay(2);
    }
}

// CAN 接收中断回调
extern "C" void can1RxCallback(can_rx_message_t *pRxMsg)
{
    j60Motor.decodeCanRxMessageFromISR(pRxMsg);
}

/**
 * @brief 传输电机控制数据
 */
inline void transmitMotorsControlData()
{
    uint32_t mailbox;
    HAL_CAN_AddTxMessage(&hcan1,
                         j60Motor.getMotorControlHeader(),
                         const_cast<uint8_t *>(j60Motor.getMotorControlData()),
                         &mailbox);
}
```

## 5：注意事项

1. **关节 ID**：确保 `MotorJ60` 构造参数与调试工具中设置的 ID 一致，否则 `decodeCanRxMessage` 无法匹配反馈帧，`isMotorConected()` 将一直返回 `false`，目前发现默认ID不一定为1，请务必自行使用串口调试助手进行设置。
2. **上电顺序**：建议先给控制板上电并启动 CAN，再给 J60 电机上电；驱动器上电默认失能，本驱动构造后默认意图为使能，随即发送 ENABLE 命令，收到应答后进入控制模式。
3. **控制周期**：建议在 `500Hz ~ 1kHz` 范围内调用闭环函数 + 发送 CAN 帧。控制周期过低会导致位置闭环响应迟钝，过高会占用 CAN 总线带宽影响多电机通信。
4. **力矩安全**：J60 最大输出扭矩 ±40 N·m，调试初期请使用较小的输出限幅，保证自身安全。
5. **板载 PD vs 板外 PID**：默认启用板外 PID（`Kp=Kd=0`，仅下发扭矩前馈），适合已有 GSRL 框架内 PID 的场景。若需要板载快速 PD 响应，调用 `hardwareMitControl()` 切换模式，调用 `exitHardwareMitMode()` 退回板外 PID 模式。
6. **掉线重连**：若 CAN 总线断开，基类连续检测到序号无更新后会将 `isMotorConected()` 置为 `false`；驱动会自动回到 `J60_DISABLED` 状态并在下一帧重发 ENABLE。CAN 恢复后几帧内即可重新使能。
7. **温度监控**：J60 以单字节复用方式上报温度，`Bit56` 标志位区分 MOSFET 温度与电机温度。使用 `getMotorTemperature()` 和 `getMosfetTemperature()` 分别读取，建议在高负载运行时轮询检查。
