# ET08ARemoteControl 使用说明（天地飞 ET08A / W.BUS）

作者：赵辰硕
最后更新日期：2026-01-17  

---

## 1：概述

`ET08ARemoteControl` 是 GSRL 中用于解析天地飞 ET08A 遥控器（接收机 W.BUS 输出）的驱动类。

它完成的工作：

- 通过 UART DMA + 空闲中断接收一帧原始数据
- 将 22 字节 payload 解包为 16 路 11bit 通道值（本类主要用到 CH1~CH8）
- 将摇杆/旋钮/微调等通道归一化为 `fp32`（近似 [-1, 1]）
- 解析 SA/SB/SC/SD 拨杆状态，并提供“跳变事件”（上/中/下互切）
- 提供掉线/连接状态判断

类定义/实现位置：

- 头文件：[GSRL/Device/inc/dvc_remotecontrol.hpp](../../Device/inc/dvc_remotecontrol.hpp)
- 源文件：[GSRL/Device/src/dvc_remotecontrol.cpp](../../Device/src/dvc_remotecontrol.cpp)

---

## 2：适用环境与依赖

- MCU：STM32
- UART 驱动：GSRL 的 `drv_uart`（内部使用 `HAL_UARTEx_ReceiveToIdle_DMA`）
- 时间基准：`HAL_GetTick()`（用于 100ms 超时判断）

建议使用方式：

- UART 接收回调（空闲中断）里只做“喂数据”到 `receiveRxDataFromISR()`
- 控制循环每次开始调用 `updateEvent()` 一次，之后读取各 getter

---

## 3：数据帧与解析说明

### 3.1 原始帧结构

本类使用如下结构体直接映射 UART 接收缓冲：

```cpp
struct ET08ARawPacket {
	uint8_t startByte;
	uint8_t data[22];
	uint8_t stopByte;
} __attribute__((packed));
```

因此“一帧长度”为 24 字节。

- `startByte`：期望为 `0x0F`
- `data[22]`：协议 payload（按 11bit packed 方式编码）
- `stopByte`：掉线判断用；当前实现中当其为 `0x0F` 时认为遥控器掉线

注意：`ET08ARemoteControl::receiveRxDataFromISR()` 不检查 `Length`，仅把指针当作 `ET08ARawPacket*` 使用；因此回调里必须自行保证拿到的是“完整 24 字节的一帧”。

### 3.2 通道解包与数值范围

`parseET08AProtocol()` 会把 `data[22]` 解包为 `temp_ch[16]`，每路为 11bit（0~2047）。

根据当前实现的经验范围：

- ET08A 通道值范围约为 `[353, 1694]`
- 中点约为 `1024`

摇杆/旋钮/微调归一化：

- 当前代码使用 `normalized = (ch - 1024) / 671.0f`
- 因此范围近似 `[-1.0, 0.9985]`

如果你需要严格保证输出落在 [-1, 1]，请在上层自行做 `clamp`。

---

## 4：快速接入步骤（推荐流程）

### 4.1 初始化 UART 接收（DMA + Idle）

工程中 UART 接收由 `UART_Init()` 启动（见 [GSRL/Driver/inc/drv_uart.h](../../Driver/inc/drv_uart.h)）。

对于 ET08A，请把 `rxDataLimit` 至少设为 24，推荐直接设为 24：

```cpp
UART_Init(&huartX, ET08AITCallback, 24);
```

### 4.2 在 UART 回调中喂数据

你需要实现一个回调函数（会在 `HAL_UARTEx_RxEventCallback` 里被调用）。强烈建议在回调里做长度校验：

```cpp
extern "C" void ET08AITCallback(uint8_t *Buffer, uint16_t Length)
{
	if (Buffer == nullptr) return;                // drv_uart 错误回调会传 NULL
	if (Length != 24) return;                     // 必须是一帧完整数据
	et08a.receiveRxDataFromISR(Buffer);
}
```

原因：当前 `ET08ARemoteControl` 把 `Buffer` 直接按 `ET08ARawPacket` 解读，长度不足会导致“解析到旧数据/随机数据”，严重时可能引发 HardFault 风险。

### 4.3 在控制循环里更新事件并读取数据

建议在每个控制周期开头调用一次：

```cpp
et08a.updateEvent();

// 再读取摇杆/开关/旋钮等
auto lx = et08a.getLeftStickX();
auto sa = et08a.getSwitchSA();
auto ev = et08a.getSwitchEventSA();
```

连接状态判断：

```cpp
if (!et08a.isConnected()) {
	// 遥控器掉线或数据过时
}
```

---

## 5：构造与通道配置（Config）

### 5.1 两种构造方式

```cpp
ET08ARemoteControl et08a(0.05f);                // 使用默认 Config + 5% 摇杆死区
ET08ARemoteControl et08a(customConfig, 0.05f);  // 自定义通道映射 + 5% 摇杆死区
```

工程示例（见 [Task/src/tsk_test.cpp](../../../Task/src/tsk_test.cpp)）：

- 推荐用 lambda 生成 `Config`，可读性更好

### 5.2 默认通道映射

默认 `Config{}`：

- 右摇杆：J1X -> CH1，J2Y -> CH3
- 左摇杆：J3Y -> CH2，J4X -> CH4
- 复用拨杆：`switchSASB -> CH5`，`switchSCSD -> CH6`
- 旋钮：`knobLD -> CH7`，`knobRD -> CH8`
- 独立 SA/SB/SC/SD、微调 T1~T4 默认均为 `CH_NONE`（即按中值 1024 处理）

> `CH_NONE` 的语义：该通道视作恒定中值 1024。

### 5.3 遥控器快速配置默认通道映射

1. 点击HOME键进入主菜单, 选择“系统设置”菜单, 选择数据重置->恢复出厂设置
2. 再次进入主菜单, 选择“通用功能”菜单, 选择“通道设置”, 按如下表格配置：
    | 通道 | 控制 | 微调 |
    | ---- | ---- | ---- |
    | 1副翼 | J1 | T1 |
    | 2升降 | J3 | T3 |
    | 3油门 | J2 | T2 |
    | 4方向 | J4 | T4 |
    | 5起落架 | SA | SB |
    | 6辅助1 | SD | SC |
    | 7辅助2 | LD | -- |
    | 8辅助3 | RD | -- |
3. 返回“通用功能”菜单, 选择“正反设置”, 按如下表格配置：
    | 通道 | 设置 |
    | ---- | ---- |
    | 1副翼 | 反 |
    | 2升降 | 反 |
    | 3油门 | 反 |
    | 4方向 | 反 |
    | 5起落架 | 正 |
    | 6辅助1 | 正 |
    | 7辅助2 | 正 |
    | 8辅助3 | 正 |

### 5.4 复用通道 vs 独立通道

ET08A 支持两种接法：

1) 复用通道（默认）：

- `switchSASB != CH_NONE` 时，SA 与 SB 来自同一通道 `switchSASB`
- `switchSCSD != CH_NONE` 时，SC 与 SD 来自同一通道 `switchSCSD`

在复用模式下：

- SA/SD 只有两挡（UP/DOWN），由通道值是否小于 1024 判断
- SB/SC 为三挡（UP/MIDDLE/DOWN），由代码中的分段规则计算（与遥控器输出刻度相关）

2) 独立通道：

- 把 `switchSASB` 设为 `CH_NONE`，并分别配置 `switchSA`、`switchSB`
- 把 `switchSCSD` 设为 `CH_NONE`，并分别配置 `switchSC`、`switchSD`

示例：

```cpp
ET08ARemoteControl::Config cfg;
cfg.switchSASB = ET08ARemoteControl::ET08AChannelIndex::CH_NONE;
cfg.switchSCSD = ET08ARemoteControl::ET08AChannelIndex::CH_NONE;
cfg.switchSA   = ET08ARemoteControl::ET08AChannelIndex::CH_5;
cfg.switchSB   = ET08ARemoteControl::ET08AChannelIndex::CH_6;
cfg.switchSC   = ET08ARemoteControl::ET08AChannelIndex::CH_7;
cfg.switchSD   = ET08ARemoteControl::ET08AChannelIndex::CH_8;

ET08ARemoteControl et08a(cfg, 0.05f);
```

运行时改映射：

```cpp
et08a.setConfig(cfg);
```

---

## 6：接口与语义说明

### 6.1 摇杆/旋钮/微调

- `getRightStickX/Y()`、`getLeftStickX/Y()`：返回归一化后的摇杆值，并自动应用死区
- `getKnobLD/RD()`：旋钮归一化值
- `getTrimmerT1~T4()`：微调归一化值（仅当 Config 映射到有效通道时才有意义）

### 6.2 SA/SB/SC/SD 拨杆状态

- `getSwitchSA()` / `getSwitchSD()`：两挡，返回 `RemoteControl::SwitchStatus2Pos`
- `getSwitchSB()` / `getSwitchSC()`：三挡，返回 `RemoteControl::SwitchStatus3Pos`

注意：这些 getter 直接返回“已缓存的状态”，不会自动解码；请先调用 `updateEvent()`。

### 6.3 跳变事件（Event）

事件通过 “当前状态” 与 “上一次状态” 的差值计算得到，因此必须保证：

- 在控制循环里每周期调用一次 `updateEvent()`
- 在读取事件前不要多次调用 `updateEvent()`（否则“上一次状态”会被刷新）

可用事件：

- `getSwitchEventSA()` / `getSwitchEventSD()`：返回 `SwitchEvent2Pos`
- `getSwitchEventSB()` / `getSwitchEventSC()`：返回 `SwitchEvent3Pos`

当发生异常（例如未更新、状态非法）时，事件可能返回 `*_ERROR` 枚举值。

### 6.4 连接状态（isConnected）

`isConnected()` 内部会调用 `decodeRxData()`，连接判断规则来自当前实现：

- 超过 100ms 未收到新数据：判定掉线
- `startByte != 0x0F`：判定无效帧/掉线
- `stopByte == 0x0F`：判定遥控器掉线（ET08A 特性）

---

## 7：典型用法示例（基于工程现有写法）

参考 [Task/src/tsk_test.cpp](../../../Task/src/tsk_test.cpp)，ET08A 的完整接入通常长这样：

```cpp
#include "dvc_remotecontrol.hpp"
#include "drv_uart.h"

ET08ARemoteControl et08a([](){
	ET08ARemoteControl::Config cfg;
	// 在这里按你的接收机通道输出配置改映射
	return cfg;
}(), 0.05f);

extern "C" void ET08AITCallback(uint8_t *Buffer, uint16_t Length)
{
	if (Buffer == nullptr) return;
	if (Length != 24) return;
	et08a.receiveRxDataFromISR(Buffer);
}

void app_init()
{
	UART_Init(&huartX, ET08AITCallback, 24);
}

void control_loop_1ms()
{
	et08a.updateEvent();

	if (!et08a.isConnected()) {
		// 安全处理：置零、刹车、进入保护模式等
		return;
	}

	float vx = et08a.getRightStickX();
	float vy = et08a.getRightStickY();

	auto sa  = et08a.getSwitchSA();
	auto esa = et08a.getSwitchEventSA();
	(void)sa; (void)esa;
}
```

---

## 8：常见问题与注意事项

1) 为什么回调里要判断 `Length == 24`？

- 因为 `receiveRxDataFromISR()` 不校验长度；长度不对时，本类仍会把 `Buffer` 当作完整帧去解包。

2) 为什么“上电后第一次读取”可能有风险？

- 当前 `decodeRxData()` 的实现中，会读取 `m_originalRxDataPointer->startByte`；如果你在从未收到过数据时就调用 getter/`isConnected()`，存在空指针风险。
- 规避方法：确保 UART 已初始化并至少收到一帧后再进入控制逻辑；或在回调喂到数据前不调用相关接口。

3) UART 配置怎么设？

- 以你接收机的 W.BUS 输出规格为准（波特率/校验/停止位/是否反相）。
- 若你使用的是“类 SBUS 输出”，常见配置为 `100000bps, 8E2`，且可能需要信号反相；但不同设备差异较大，请优先以说明书为准。

4) 事件跳变不准/总是 `NO_CHANGE`？

- 确认每个控制周期只调用一次 `updateEvent()`。
- 确认你的回调每帧都有喂到 `receiveRxDataFromISR()`，且 Length 恒为 24。

5) 输出值不到 1.0 是正常的吗？

- 是正常的。当前归一化使用 671 作为偏移量上限，正方向最大约为 0.9985。
