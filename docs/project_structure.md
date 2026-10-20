# 工程结构与定位来源

本次整理将硬件接收、设备协议、定位、模式控制和算法分开。当前选择雷达，里程计与雷达参数独立配置。

## 目录职责

```text
Core/                         CubeMX 入口、初始化和中断
Drivers/                      STM32 驱动
Middlewares/                  FreeRTOS、USB 中间件
USB_DEVICE/                   CubeMX USB 接口；回调将字节交给 BSP
MDK-ARM/                      Keil、EIDE 工程配置
bsp/                          CAN、UART、USB 字节收发及 DWT 计时
protocol/
    vesc/                     VESC 协议和设备状态
    dji/                      DJI 6020 协议和设备状态
    remote/                   遥控器接收解析
    imu/                      超核 IMU 接收解析
    lidar/                    雷达位姿拼帧、校验、解析
localization/                 里程计、雷达车体中心换算、位姿来源管理
control/
    manual/                   遥控输入到车体速度的映射
    auto/                     航点流程和单段 AutoMove 控制
algorithm/
    trajectory/               梯形速度规划
    kinematics/               底盘运动学正逆解
    pid/                      通用 PID
    math/                     数学与单位换算
chassis/                      底盘反馈、电机控制
common/                       共用位姿、速度和反馈类型
robot_config/                 机械、控制、航点和定位配置
debug/                        Watch 镜像，不参与控制
Task/                         底盘任务与周期调度
tests/                        主机行为测试，使用实际解析、定位和控制代码
```

## 常用修改入口

| 文件 | 修改内容 |
|---|---|
| `robot_config/robot_config.h` | 底盘机械参数、电机 ID、手动速度、舵向 PID |
| `robot_config/auto_move_config.h` | `AutoMoveConfig::Odometry`、`AutoMoveConfig::Lidar` 两套自动运动参数 |
| `robot_config/auto_route_config.h` | 车体中心的绝对目标 x/y 和 yawDeg，按数组顺序跑点 |
| `robot_config/localization_config.h` | 位姿来源、雷达超时、安装偏移、USB 每拍读取上限 |

例如，选择雷达时修改：

```cpp
constexpr PoseSource POSE_SOURCE = PoseSource::Lidar;
```

修改后重新编译。来源在底盘任务初始化时确定，自动模式每次复位时绑定对应参数，不在行进中切换。雷达无效不会回退到里程计。

两种来源共用 AutoMove 计算逻辑，规划速度、加减速度、P/D 增益、输出限制和到达条件均取自对应参数组。里程计整改前的 16 项参数已逐项核对保留，其中加速度为 0.6 m/s²、减速度为 0.4 m/s²、沿路径 Kd 为 1.2。

## 底盘周期与位姿接口

底盘任务依次更新电机反馈、更新选中的定位、选择运行模式、计算目标速度、执行底盘控制、更新调试镜像。

- 选择里程计：启动并更新 IMU，使用实测车体速度和 IMU 航向积分里程计。
- 选择雷达：不启动 IMU 接收、不更新 IMU、不积分里程计；位置和航向完全来自雷达。
- 两种来源都保留轮速和舵角反馈，用于实测速度、沿路径 D 反馈及停稳判断。
- 所有模式下都限量处理雷达 USB 数据，便于停机或手动模式时检查接收情况。

`PoseSourceManager` 提供三个只读接口：

```cpp
bool GetOdometryPose(PoseFeedback &feedback) const;
bool GetLidarPose(PoseFeedback &feedback) const;
bool GetCurrentPose(PoseFeedback &feedback) const;
```

返回值表示反馈是否有效。即使无效，输出快照中也保留来源和状态；自动控制必须检查有效性。Get 函数不接收数据、不等待、不推进积分。

`ChassisPose2D`、`ChassisBodyVelocity` 和 `PoseFeedback` 定义在 `common/motion_types.h`。位置计算使用 m，航向使用 rad，角速度使用 rad/s；角度配置和 Watch 使用度，角速度 Watch 使用 °/s。

## USB 接收与雷达协议

```text
CDC_Receive_HS
    → BspUsb_ReceiveBytes：缓存字节及接收时间
    → Chassis_Task 调用 PoseSourceManager::UpdateLidar()
    → LidarLocalization::Update()
        → BspUsb::Read：每拍最多取 USB_READ_BUDGET_BYTES 个字节
        → LidarPoseProtocol::FeedByte：拼帧、校验、解析
        → 雷达中心换算为车体中心
    → GetCurrentPose
    → AutoModeController / AutoMove
```

不新建 USB 任务，`StartDefaultTask()` 不接收或解析 USB。USB 初始化由底盘任务中的 `BspUsb::Init()` 调用一次。没有数据立即返回，半帧保留到下一拍；不能添加等待整帧或不断清空缓存的循环。

当前协议保持为 15 字节：

| 字节 | 内容 |
|---|---|
| 0–1 | 帧头 `AA 55` |
| 2–5 | x，小端 IEEE754 float32，m |
| 6–9 | y，小端 IEEE754 float32，m |
| 10–13 | yaw，小端 IEEE754 float32，rad |
| 14 | 前 14 字节累加和的低 8 位 |

USB 断连、重新枚举或缓存溢出时清除协议半帧并使旧雷达反馈无效。坏帧和非有限浮点值不更新位姿。定位超时依据完整帧最后一个字节的 USB 接收时间，解析积压数据不会刷新其接收时间；该时间不包含雷达算法和上位机发送前的延迟。

## 雷达中心到车体中心

雷达定位参考点使用实测旋转中心。安装偏移为车体坐标系中“旋转中心指向雷达中心”的向量：前方 0.4636 m、左方 0.0160 m。yaw 偏移为雷达坐标轴相对车体坐标轴的角度，当前为 0°。

```text
yawChassis = yawRadar - yawMount
centerWorld = pRadar - R(yawChassis) * offsetBody
currentPosition = centerWorld - initialCenterWorld
```

旋转中心的世界位置计算如下：

```cpp
centerWorldX = radarX - 0.4636f * cosf(yaw) + 0.0160f * sinf(yaw);
centerWorldY = radarY - 0.4636f * sinf(yaw) - 0.0160f * cosf(yaw);
```

定位 Reset 后，首次完整、未超时且换算有效的雷达帧记录 initialCenterWorld，之后只减去该固定位置。原点在所有底盘模式下均可建立，不等待进入自动档；建立时车辆应停稳且小电脑定位已稳定。帧有效只表示协议、数值和新鲜度满足要求，并不自动判断定位是否稳定。

跑点坐标轴保留雷达世界 x/y 方向，航向保留安装角修正后的世界航向，不旋转坐标轴、不减初始 yaw。初始雷达为 `(0,0,0)` 时，记录的中心世界位置为 `(-0.4636,-0.0160)`，输出为 `(0,0,0)`；初始 yaw 非零时输出保留该航向。

航点 `{5.0f,0.0f,0.0f}` 表示旋转中心沿世界 +x 相对原点移动 5 m；随后 `{5.0f,0.0f,90.0f}` 表示保持同一目标位置转到 90°。不需要逐个换算航点。里程计仍使用原有局部坐标。

超时、USB 断连/溢出、手动/自动切换以及航点切换均保留原点；只有定位 Reset 才清除原点。小电脑中途重新设置地图原点时，须重新初始化单片机定位并重新开始路线，不能沿用旧原点。

本次只修改定位参考点与位置归零；现有几何中心运动学及轮速反馈未加入 O/G 速度换算。实测旋转中心与运动学模型之间的差异仍须实车验证，定位归零不代表已解决该差异。

连续两个航点的 x/y 相同、仅 yawDeg 不同时，目标车体中心保持不动。实际未到达该中心时，位置反馈仍会修正残余误差。

## Watch 入口

统一展开 `g_debugSnapshot`：

| 变量 | 含义 |
|---|---|
| `usbReceive` | USB 连接、收包、缓存积压和溢出统计 |
| `localization.selectedSource` | 当前选择的定位来源 |
| `localization.currentPose` | 自动控制实际采用的车体中心位姿、时间和有效性 |
| `localization.odometryPose` | 里程计反馈；雷达模式下不积分且保持无效 |
| `localization.lidarPose` | 相对固定起点的旋转中心反馈；yaw 为世界航向 |
| `localization.lidarOrigin` | initialized 及初始旋转中心世界坐标 initialCenterWorldX/Y |
| `localization.lidarProtocol` | 原始雷达中心位姿、有效帧计数、校验错误及最后一帧 |
| `autoRoute.poseSource` | 自动航点任务绑定的参数来源 |
| `autoMove` | 梯形参考、位置和速度反馈、输出与运动阶段 |
| `measuredVelocity` | 底盘实测车体速度 |

旧的 `g_usb_pose_debug`、`g_usb_rx_debug` 由上述镜像替代。Watch 中 `yawDeg` 为度，`wzDegPerSec` 为 °/s。

## 验证与 CubeMX 再生成

原点转换通过临时主机测试验证，包含首次有效帧、平移、旋转、非零初始航向、分包连包、超时/断连恢复及显式复位。工程中的 tests 文件目前已删除，未恢复；主机测试不替代实车定位精度和控制周期耗时验证。

Keil/EIDE 的应用分组和头文件路径均已同步迁移。以后通过 CubeMX 重新生成时，需要核对应用源文件和 include 路径仍在工程中，并检查 `freertos.c` 的 `MX_USB_DEVICE_Init()` 是否又被自动插入默认任务；USB 仍应由 BSP 在底盘初始化阶段统一初始化一次。
