# BMS-2.0

电池管理系统（BMS 节点）——**STM32G0B1 + TI BQ76952 + SC8815** 方案，Cortex-M0+ 平台，FDCAN 总线通信，FreeRTOS 多任务。

相比初代 5 串方案（BQ76920 + STM32F103），本版本升级为：AFE 换用带内置电量计与硬件保护的 **BQ76952**，新增 **SC8815** 充电管理芯片与 **FDCAN** 对外通信，构成完整的电池管理节点。

## 硬件平台

| 部件 | 型号 | 职责 |
|---|---|---|
| 主控 | STM32G0B1CBT6（Cortex-M0+） | CubeMX 生成 HAL 框架，片上 FDCAN |
| AFE | TI BQ76952 | 3~16 节监视：电压/电流/温度采样、内置硬件保护（OV/UV/OCD/SCD/OT/UT）、库仑计 SOC |
| 充电管理 | SC8815 | I2C 控制的升降压充放电管理 |
| 通信 | FDCAN | 电池节点 ↔ 网关节点的报文交互 |
| 显示 | OLED（SSD1306） | SOC 与状态显示 |

## 软件架构

```
┌────────────────────────────────────────────────────┐
│ App 层                                             │
│  App_Main.c            任务创建与调度               │
│  App_Battery_Manager   电池管理（AFE/显示/充电）    │
│  App_Battery_Control   电池控制（充放电策略）       │
├────────────────────────────────────────────────────┤
│ Com 层（纯算法，不碰硬件）                          │
│  Com_SOC               电量估算                    │
│  Com_BatteryParam      电芯参数（OCV-SOC 曲线表）   │
├────────────────────────────────────────────────────┤
│ Int 层（设备驱动）                                  │
│  Int/BQ76952           AFE 驱动 + 寄存器位域定义    │
│  Int/SC8815            充电芯片驱动                │
│  Int/CAN               FDCAN 报文收发              │
│  Int/OLED              显示驱动 + 界面渲染          │
├────────────────────────────────────────────────────┤
│ Core 层  CubeMX 生成：GPIO / I2C / FDCAN / HAL     │
└────────────────────────────────────────────────────┘
```

FreeRTOS 任务划分：

| 任务 | 职责 |
|---|---|
| `__battery_manager_task` | 周期性采集与状态管理（AFE 数据、OLED 刷新、SC8815 控制） |
| `__battery_control_task` | 充放电控制策略 |
| `__battery_remote_task`  | FDCAN 远程通信（对外上报/响应） |

## 关键技术点

### 1. BQ76952 驱动

- 寄存器子命令协议：`0x3E/0x3F` 命令通道写入子命令，`0x40~0x45` 交换区读取返回数据（含校验和），封装成通用的 `subcmd_read/subcmd_write`
- 出厂配置写入（保护阈值、电量计参数）走 `Data Memory` 访问流程：解锁 → 写入 → 校验 → 上锁
- SOC 直接读取 AFE 内置电量计结果，`Com_SOC` 层用 OCV-SOC 表（EVE 50E 电芯，25 °C NMC/NCA 曲线细化到 1%）做校验与开机起点，表数据预留了按实际电芯静置标定的接口

### 2. SC8815 充放电管理

I2C 配置充电电流/电压/系统电压寄存器，配合 BQ76952 的 FET 控制实现完整充放电路径管理。

### 3. FDCAN 通信

- 经典 CAN / FDCAN 帧封装，报文 ID 按功能域划分
- `battery_remote_task` 独立处理对外通信，与管理/控制任务解耦

## 工程结构

```
├── Core/                      # CubeMX 生成
├── Drivers/                   # ST HAL + CMSIS
├── MDK-ARM/
│   ├── bms-project.uvprojx    # Keil 工程
│   ├── source/
│   │   ├── App/               # 任务与业务调度
│   │   ├── Com/               # SOC/电芯参数算法
│   │   └── Int/               # BQ76952 / SC8815 / CAN / OLED 驱动
│   └── thirdpart/FreeRTOS/
└── bms-project.ioc            # CubeMX 配置
```

## 编译与烧录

1. **Keil MDK** 打开 `MDK-ARM/bms-project.uvprojx`
2. Build → Download（目标 STM32G0B1）
3. OLED 上电显示 `BMS项目` 标题与实时 SOC

## 当前状态

- [x] BQ76952 驱动（子命令协议、Data Memory 配置、SOC/电压/电流读取）
- [x] SC8815 充电管理驱动
- [x] FDCAN 收发框架
- [x] OLED 界面
- [ ] 充放电策略与 CAN 报文协议的完整业务闭环

## 许可

仅供学习交流使用。
