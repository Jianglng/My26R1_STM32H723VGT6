# My26R1 · STM32H723 三舵轮底盘控制

基于 **STM32H723VGT6** 的三舵轮全向底盘固件，使用 C++14、STM32 HAL 和 FreeRTOS，实现遥控驾驶、自动航点运动，以及雷达或轮速里程计定位。

底盘由三个独立转向与驱动的轮组组成：DJI 6020 负责舵向，VESC 电调负责轮速。工程将设备通信、定位、运动控制和纯算法分层组织，并提供可在 PC 上运行的行为测试。

## 功能

- **三舵轮运动学**：根据车体速度计算各轮舵角和转速，根据舵角、轮速反馈估计车体速度；舵向误差超过 90° 时反转轮速，减少转向行程。
- **电机控制**：舵向使用角度环与速度环串级 PID，驱动轮通过 CAN 下发 VESC 转速指令。
- **遥控驾驶**：支持平移与旋转，提供摇杆死区、方向和速度上限配置。
- **自动航点**：梯形速度规划结合沿路径 PD、横向 P 和航向 P 控制；位置、航向及实测速度持续满足到达条件后，才进入下一航点。
- **两种定位来源**：USB 雷达位姿，或轮速反馈与超核 IMU 融合的里程计；通过编译期配置选择。
- **异常处理**：遥控离线、电机反馈失效时停止输出；自动运动支持定位失效等故障的锁存处理。
- **调试与验证**：集中 Watch 入口、RAM 在线调参，以及覆盖协议、定位、规划和控制行为的主机测试。

## 硬件与接口

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32H723VGT6，Cortex-M7 |
| 时钟 | 外部晶振 24 MHz，系统时钟 480 MHz |
| 系统调度 | FreeRTOS，底盘任务目标周期 1 ms |
| 驱动电机 | VESC CAN 扩展帧，FDCAN1，500 kbit/s |
| 舵向电机 | DJI 6020 CAN 标准帧，FDCAN3，1 Mbit/s |
| 遥控接收 | UART5，100000 baud，8 位数据、偶校验、2 位停止位，18 字节遥控帧 |
| IMU | 超核 IMU，USART10，115200 baud；选择里程计时启用接收 |
| 雷达定位输入 | USB CDC，接收上位机发送的位姿帧 |

当前 MCU 引脚分配如下，板上连接器位置以实际硬件为准：

| 接口 | RX / DM | TX / DP |
| --- | --- | --- |
| FDCAN1 | PD0 | PD1 |
| FDCAN3 | PD12 | PD13 |
| UART5 | PD2 | PC12（当前仅启用 RX） |
| USART10 | PE2 | PE3 |
| USB | PA11（DM） | PA12（DP） |

CAN 引脚需通过板上的 CAN 收发器连接总线。VESC 需配置与固件一致的波特率、节点号，并启用包含转速的 `Status` 状态帧广播；主控依赖该反馈判断驱动轮是否在线。

轮组顺序统一为 **前轮、左轮、右轮**：

| 参数 | 前轮 `[0]` | 左轮 `[1]` | 右轮 `[2]` |
| --- | --- | --- | --- |
| VESC 节点 ID | 90 | 69 | 97 |
| DJI 6020 电机 ID | 1 | 4 | 3 |
| 舵向零点编码器值 | 6922 | 7180 | 5077 |

当前轮径为 `0.120 m`，底盘中心到轮心距离为 `0.36667 m`，驱动减速比为 `1.0`，VESC 电机极对数为 `14`。移植到其他底盘时，在 [robot_config.h](robot_config/robot_config.h) 中配置机械尺寸、电机 ID、零点和控制参数。

## 工程结构

```text
.
├── task/                                   底盘任务、应用对象与周期调度
│   ├── chassis_task.h / .cpp                FreeRTOS 底盘任务入口与固定周期调度
│   └── chassis_app.h / .cpp                 初始化、反馈、定位、模式选择与控制流程
├── chassis/                                电机反馈、串级 PID 与底盘输出
│   └── chassis.h / .cpp                     三轮反馈汇总、舵向 PID 和驱动轮转速输出
├── control/                                运行模式与运动控制
│   ├── manual/                             手动控制
│   │   └── manual_mode_controller.h / .cpp  遥控输入到车体速度的映射
│   └── auto/                               自动控制
│       ├── auto_mode_controller.h / .cpp    航点顺序、启动条件与路线完成状态
│       └── auto_move.h / .cpp               单段轨迹跟踪、误差修正与到达判断
├── localization/                           定位与位姿来源管理
│   ├── chassis_odometry.h / .cpp            轮速、舵角与 IMU 航向积分里程计
│   ├── lidar_localization.h / .cpp          雷达输入、安装偏移换算与位置原点
│   └── pose_source.h / .cpp                 里程计、雷达及当前选定来源的统一接口
├── algorithm/                              纯算法
│   ├── kinematics/                         三舵轮运动学正逆解
│   │   └── chassis_kinematics.h / .cpp      车体速度与轮组舵角、转速之间的换算
│   ├── math/                               数学工具与单位换算
│   │   └── math_utils.h                    限幅、死区、角度归一化与单位换算
│   ├── pid/                                通用 PID
│   │   └── pid.h / .cpp                     PID 计算、编码器周期误差与状态管理
│   └── trajectory/                         轨迹规划与运动参数
│       ├── trapezoidal_profile.h / .cpp     梯形速度规划与参考状态
│       └── auto_move_params.h              自动运动参数类型与有效性检查
├── protocol/                               设备协议与反馈解析
│   ├── dji/                                DJI 舵向电机协议
│   │   └── dji_6020.h / .cpp                6020 电流指令与编码器、转速反馈
│   ├── imu/                                超核 IMU 协议
│   │   ├── chaohe_imu.h / .cpp              IMU 接收封装、姿态反馈与在线状态
│   │   ├── hipnuc_dec.h / .c                厂商协议解码
│   │   └── hipnuc_sample.h / .c             厂商数据处理辅助代码
│   ├── lidar/                              雷达位姿协议
│   │   └── lidar_pose_protocol.h / .cpp     15 字节位姿帧拼接、校验与解析
│   ├── remote/                             遥控器协议
│   │   └── remote_input.h / .cpp            遥控帧接收、通道解码与在线判断
│   └── vesc/                               VESC 电调协议
│       └── vesc_motor.h / .cpp              CAN 控制组帧与转速、电流等状态解析
├── bsp/                                    板级硬件封装
│   ├── bsp_can.h / .cpp                     CAN 收发缓存、回调分发与总线恢复
│   ├── bsp_uart.h / .cpp                    UART DMA 接收与回调分发
│   ├── bsp_usb.h / .cpp                     USB 初始化、接收缓存与接收时间记录
│   ├── bsp_dwt.h / .cpp                     DWT 计时与实际时间间隔测量
│   └── bsp_critical.h                      临界区封装
├── robot_config/                           车型与控制配置
│   ├── robot_config.h                      机械尺寸、电机 ID、舵向零点与 PID 默认值
│   ├── localization_config.h               定位来源、雷达安装偏移与超时
│   ├── auto_route_config.h                 自动航点表
│   ├── auto_move_config.h                  自动运动默认参数与调参输出上限
│   └── tune_params.h / .cpp                RAM 在线调参候选值与应用结果
├── common/                                 共用位姿、速度和反馈类型
├── debug/                                  调试快照与 Watch 入口
├── Core/                                   CubeMX 生成的入口、初始化与中断
├── Drivers/                                CMSIS 与 STM32 HAL
├── Middlewares/                            FreeRTOS 与 USB 中间件
├── USB_DEVICE/                             USB CDC 设备接口
├── MDK-ARM/                                Keil 与 EIDE 工程
└── tests/                                  PC 主机行为测试
    └── stubs/                              主机测试使用的硬件接口替身
```

底盘任务由 [chassis_task.cpp](task/chassis_task.cpp) 调度，[ChassisApp](task/chassis_app.cpp) 组织每个控制周期：

```text
更新电机反馈 → 处理雷达输入 → 更新选定定位 → 选择模式
    → 生成车体目标速度 → 运动学与电机控制 → 刷新调试快照
```

控制周期使用 `vTaskDelayUntil()` 调度，实际 `dt` 通过 DWT 测量，并传入定位、自动控制和 PID 算法。

## 编译

### 开发环境

| 工具 | 工程配置 |
| --- | --- |
| Keil MDK / µVision | ARM Compiler 6，工程记录版本为 6.24 |
| STM32 器件包 | `Keil.STM32H7xx_DFP.4.1.3` |
| C / C++ 标准 | C99 / C++14 |
| STM32CubeMX | `.ioc` 记录版本为 6.18.1 |
| STM32CubeH7 | `.ioc` 记录固件包为 V1.13.0 |
| VS Code + EIDE | 使用工程内的 AC6 配置 |

HAL、CMSIS、FreeRTOS 和 USB 中间件源码已包含在仓库中。直接编译已有工程时无需先运行 CubeMX。

### Keil MDK

1. 安装 ARM Compiler 6 和对应 STM32H7 器件包。
2. 打开 [My26R1_STM32H723VGT6.uvprojx](MDK-ARM/My26R1_STM32H723VGT6.uvprojx)。
3. 选择 `My26R1_STM32H723VGT6` Target，检查本机编译器路径。
4. 执行 **Rebuild** 生成固件。

### VS Code + EIDE

1. 安装 EIDE 扩展，并配置 ARM Compiler 6 的本机路径。
2. 通过 EIDE 打开 `MDK-ARM` 下的工程，配置文件为 [eide.yml](MDK-ARM/.eide/eide.yml)。
3. 选择 `My26R1_STM32H723VGT6` 目标，执行构建。

新增源文件或修改包含路径时，需要同步 Keil 和 EIDE 工程配置。使用 CubeMX 再生成代码后，还需核对应用文件是否保留，并确认 USB 仍由 `BspUsb::Init()` 初始化一次，避免默认任务再次调用 `MX_USB_DEVICE_Init()`。

## 运行与配置

### 遥控模式

左拨杆选择底盘运行模式：

| 左拨杆 | 模式 | 行为 |
| --- | --- | --- |
| 下档 | `Stop` | 驱动轮转速指令与舵向电流指令清零 |
| 中档 | `Manual` | 左纵轴控制 `vx`，左横轴控制 `vy`，右横轴控制 `wz` |
| 上档 | `Auto` | 从航点表首点开始执行自动路线 |

初始化失败或遥控离线时进入 `Stop`。自动路线完成后保持停止；重新进入自动档会从首个航点重新执行。自动运动进入 `Fault` 后，需要切换模式使控制器复位，反馈恢复本身不会解除锁存故障。

首次运行前，将左拨杆置于下档，并核对电机 ID、转向零点和轮速反馈。当前手动速度上限为 `vx/vy = 1.5 m/s`、`wz ≈ 85.94°/s`；自动运动使用独立参数，见 [auto_move_config.h](robot_config/auto_move_config.h)。

### 配置入口

| 文件 | 配置内容 |
| --- | --- |
| [robot_config.h](robot_config/robot_config.h) | 电机 ID、轮径、轮心位置、舵向零点、手动速度与舵向 PID |
| [localization_config.h](robot_config/localization_config.h) | 定位来源、雷达安装偏移、超时与 USB 每周期处理预算 |
| [auto_move_config.h](robot_config/auto_move_config.h) | 雷达和里程计各自的自动运动参数、输出限制与到达条件 |
| [auto_route_config.h](robot_config/auto_route_config.h) | 按顺序执行的目标位姿表 |
| [tune_params.h](robot_config/tune_params.h) | 在线调参候选值与应用结果 |

### 坐标与定位

车体坐标约定为 **+x 向前、+y 向左、+wz 俯视逆时针**。内部位置使用 m，速度使用 m/s，航向使用 rad，角速度使用 rad/s；航点及调参中带 `Deg` 的字段使用度。

在 `localization_config.h` 中选择定位来源，修改后重新编译：

```cpp
constexpr PoseSource POSE_SOURCE = PoseSource::Lidar;
// 使用轮速 + IMU 里程计时，设为 PoseSource::Odometry
```

当前默认使用雷达。两种来源独立配置，雷达失效时不会自动切换到里程计。

- **雷达定位**：先按安装偏移将雷达参考点换算为车体旋转中心，Reset 后以首次有效帧的中心位置建立固定原点；保留雷达世界坐标轴方向和修正后的世界航向，不减去初始 yaw。
- **里程计定位**：利用实测轮速、舵角和 IMU 航向，在本地坐标系中累计位姿；选择该来源时才启动 IMU 接收与里程计更新。

雷达原点可在停机或手动模式下建立，不必等待进入自动档。建立位置原点只表示记录定位参考位置；帧有效性并不代表定位已稳定。原点在模式切换和雷达短暂离线后保留，定位 Reset 才会清除。

当前源码中的雷达安装偏移为 `x = 0.4416 m`、`y = 0 m`、`yaw = 0°`。配置值以 [localization_config.h](robot_config/localization_config.h) 中的常量为准，部分历史文档和注释记录的是较早参数。

### 自动航点

编辑 `AutoRouteConfig::POINTS`，每项格式为 `{x, y, yawDeg}`。当前路线为：

```cpp
constexpr AutoRoutePoint POINTS[] = {
    {3.0f, 0.0f, 0.0f},
    {3.0f, 0.0f, 90.0f},
    {3.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}
};
```

航点是**当前定位坐标系下的车体中心绝对目标位姿**。雷达模式下，上述路线表示沿世界 +x 到达相对原点 3 m 的位置，转到 90°，转回 0°，再返回位置原点。连续航点的 x/y 相同而航向不同时，保持同一位置目标并转向。

每个航点单独规划，控制器根据实际位置、航向、速度及持续停稳时间判断完成；规划时间结束后仍会修正残余误差。

## 雷达位姿通信协议

上位机通过 USB CDC 发送固定 **15 字节**二进制帧：

| 字节偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0–1 | 2 | 帧头 `AA 55` |
| 2–5 | 4 | 雷达 x，小端 IEEE 754 float32，m |
| 6–9 | 4 | 雷达 y，小端 IEEE 754 float32，m |
| 10–13 | 4 | 雷达 yaw，小端 IEEE 754 float32，rad |
| 14 | 1 | 前 14 字节累加和的低 8 位 |

Python 组帧示例，返回值可直接交给已打开的 USB CDC 串口写入：

```python
import struct

def pack_pose(x: float, y: float, yaw_rad: float) -> bytes:
    payload = b"\xAA\x55" + struct.pack("<fff", x, y, yaw_rad)
    return payload + bytes([sum(payload) & 0xFF])
```

接收端支持分包、连续帧和坏帧后重新同步，拒绝校验错误及 NaN/Inf 数据。默认每个底盘周期最多处理 `128` 字节，完整有效帧超过 `200 ms` 未更新则定位失效；超时使用完整帧最后一个字节的 USB 接收时间。

## 调试与在线调参

在 Keil Watch 中添加以下入口：

| 变量 | 用途与生效方式 |
| --- | --- |
| `g_debug.status` | 每 10 ms 刷新的观测快照，修改它不会影响控制 |
| `g_debug.app` | 真实应用对象指针，可展开模式、定位、底盘和 PID 状态 |
| `g_autoMoveTune.lidar` / `.odometry` | 自动运动参数候选值；切换模式时校验并应用，行进中修改不会立即生效 |
| `g_steerPidTune.angle` / `.speed` | 三个轮子共用的舵向 PID 候选参数；仅在 `Stop` 模式每周期应用 |

自动运动调参通过 `lastApplyValid`、`applyCount`、`rejectCount` 和 `applied` 查看应用结果；舵向 PID 调参通过 `g_steerPidTune.lastApplyValid` 查看最近一次应用是否成功。结果回显字段不作为控制输入。

RAM 调参在复位或断电后恢复默认。需要保留的参数应写回 `auto_move_config.h` 或 `robot_config.h` 中的默认值。定义 `DEBUG_SNAPSHOT_ENABLE=0` 可关闭调试快照刷新和应用对象入口登记。

## 主机测试

测试使用工程中的实际 PID、运动学、轨迹规划、自动控制、定位和电机协议实现，以替身提供硬件收发和时钟。需要 PowerShell 与支持 C++14 的 `g++`。

在仓库根目录执行：

```powershell
./tests/run_tests.ps1
```

若编译器未加入 `PATH`，通过参数指定本机路径：

```powershell
./tests/run_tests.ps1 -Compiler 'C:\path\to\mingw64\bin\g++.exe'
```

脚本使用 `-Wall -Wextra -Werror -pedantic` 编译，随后运行测试；编译或测试失败会报错。测试覆盖控制状态转换、闭环运动仿真、规划阶段、雷达帧与原点换算、定位超时和计数回绕，以及底盘反馈离线与 CAN 输出。

主机测试用于验证软件行为；真实中断、DMA、控制周期耗时、定位精度和机械运动仍需在目标硬件上验证。测试详情见 [tests/README.md](tests/README.md)。

## 第三方组件许可

STM32 HAL、CMSIS、FreeRTOS、USB 中间件等第三方组件的许可见各自目录中的许可证文件。
