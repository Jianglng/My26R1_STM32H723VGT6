# 代码规范评审：问题清单与修改建议

对照 [coding_standard.md](coding_standard.md) 评审应用层代码，CubeMX 生成代码和 `hipnuc_*` 第三方代码不在范围内。评审基于 2026-10-29 的工作区，包含尚未提交的修改。

优先级：

- **P0**：可能影响安全或导致行为错误，先修。
- **P1**：结构和冗余问题，是代码显得混乱的主要原因。
- **P2**：命名、格式、注释，可以用工具批量处理。
- **P3**：工程化配套。

## 本轮实施状态（2026-10-30）

此文保留原始问题与建议，当前实现及验证结果见 [refactor_report.md](refactor_report.md)。雷达前向偏移按用户确认值设为 0.4636 m；IWDG 按用户要求暂缓。边界校验策略以更新后的编码规范 6.3 节为准，不把公开的算法接口都当作外部输入边界。

## 总体评价

底子比你感觉的要好。分层目录已经清楚，大部分类职责单一，`isfinite` 检查和异常导向零输出做得很扎实，注释量也够。显得混乱的原因主要有四个：

1. 同一件事写了好几份：常量重复定义，参数结构体重复，临界区代码重复。
2. 分层边界有几处被打穿，下层反过来依赖上层或车型配置。
3. 为了方便调试把成员设成 `public`，接口变得不干净。
4. 格式和注释没有工具约束，风格逐渐漂移。

## P0：安全与正确性

| # | 位置 | 问题 | 建议 |
|---|---|---|---|
| 1 | `robot_config/localization_config.h:14,16` | 注释写雷达在旋转中心前方 0.4636 m，实际值是 `0.51f`，两者对不上 | 确认哪个是实测值，改成一致。如果 0.51 是后来标定的结果，在提交说明里写清楚标定方法 |
| 2 | `Task/Chassis_Task.cpp:81,87` | `remote.Init()`、`imu.Init()` 返回 `HAL_StatusTypeDef`，结果被丢弃。遥控串口启动失败时，系统会一直处于"遥控离线"状态，但看不出原因 | 检查返回值，失败时记录错误码并保持停机 |
| 3 | `Core/Inc/stm32h7xx_hal_conf.h:64` | 没有启用独立看门狗。控制任务卡死时，电机会一直保持最后一帧指令 | 启用 IWDG，在底盘主循环中喂狗。另外确认 VESC 和 6020 自身在收不到指令时会不会停转。代码里的 `ONLINE_TIMEOUT_MS` 只用于主控判断反馈是否在线，不能保护电机 |
| 4 | `Task/Chassis_Task.cpp:160` | 用 `osDelay(1)` 控制周期，实际周期是"1 ms + 本拍执行时间"，会漂移 | 改用 `vTaskDelayUntil()` 固定周期。`dt` 是实测的，所以现在不会算错，但周期本身不稳定 |
| 5 | `protocol/imu/chaohe_imu.h:33` | 注释写传入 `&huart1`，实际代码传的是 `&huart10` | 改注释，或者把注释里的具体句柄删掉 |

## P1：结构与冗余

### 6. 分层依赖被打穿

| 位置 | 问题 |
|---|---|
| `algorithm/pid/pid.cpp:7,58` | 纯算法模块直接包含 `bsp_dwt.h`，读取 `DWT->CYCCNT` |
| `algorithm/kinematics/chassis_kinematics.h:5` | 算法模块包含 `robot_config.h`，轮数、轮心坐标、零点都写死在算法里 |
| `protocol/dji/dji_6020.cpp:89-110`、`protocol/vesc/vesc_motor.cpp:54` | 协议层直接改写 `BspCan::fdcan3TxFrame_` 的帧头和数据 |
| `protocol/dji/dji_6020.h:17,108`、`protocol/vesc/vesc_motor.h:13,125` | 协议层用 `RobotConfig::WHEEL_COUNT` 决定注册表大小，换车型要改协议代码 |

建议：

- **PID**：`Calculate()` 改为接收 `dt` 参数，由 `Chassis` 统一测一次 `dt`，再传给 6 个 PID。这样 PID 可以在 PC 上测试，6 个 PID 也共用同一个时间基准。
- **运动学**：构造时传入一个 `ChassisGeometry` 结构体（轮心坐标、零点、轮径），由 `Chassis` 用 `RobotConfig` 填好后交给它。
- **CAN 发送**：`BspCan` 提供 `bool Send(FDCAN_HandleTypeDef* bus, uint32_t id, IdType type, const uint8_t* data, uint8_t length)`，发送帧缓冲区改为 `private`。
- **协议注册表**：容量改为协议自己的常量，例如 `Dji6020Cfg::MOTOR_ID_MAX`（8）。

### 7. 为了调试公开数据成员

`Pid::state_`（`pid.h:78`）、`DwtTimer::sysTime_`（`bsp_dwt.h:26`）、`BspCan` 的 4 个静态帧（`bsp_can.h:31-34`）都是 `public`。Keil Watch 本来就能展开私有成员，`g_debug.obj.chassis->steerAnglePid_` 就是这样看的。改成 `private`，另外提供 `const PidState& State() const`。

### 8. 常量重复定义

| 常量 | 出现位置 |
|---|---|
| `PI` | `math_utils.h:10`、`auto_move.cpp:9`、`chassis_odometry.cpp:9`、`chassis_kinematics.cpp:9` |
| 度和弧度换算 | `MathUtils::DegToRad()`，另有 `chassis_odometry.cpp:10` 的 `DEG_TO_RAD` |
| `MAX_UPDATE_INTERVAL_S = 0.05f` | `auto_move.cpp:10`、`chassis_odometry.cpp:11` |
| 6020 编码器量程 8192 | `RobotConfig::DJI_ENCODER_MAX`、`Dji6020Cfg::ENCODER_MAX` |
| 6020 控制 ID `0x1FF` | `Dji6020Cfg::CTRL_ID_GROUP1`，另有 `bsp_can.cpp:55` 写死的数值 |

建议：`PI` 和角度换算只留在 `math_utils.h`；编码器量程属于电机协议，只留在 `Dji6020Cfg`；`0.05f` 的控制周期上限放进 `robot_config.h`，作为 `MAX_CONTROL_DT_S`。

### 9. 参数被重复写了三遍

`auto_move_config.h` 中，每个参数先定义为单独的常量（第 39-56 行），再按位置顺序填进 `ODOMETRY_DEFAULT`（第 83-103 行）。`AutoMoveTune`（`auto_move_config.h:8`）和 `AutoMoveParam`（`auto_move.h:20`）又是两份 18 个字段的结构体，中间靠 `ToAutoMoveParam()` 逐个字段复制，再用 `sizeof` 比较做 `static_assert` 兜底（`auto_mode_controller.cpp:8`）。

问题在于：按位置初始化时，两个字段顺序写反也能编译通过；`sizeof` 相等也不能说明字段一一对应。

建议：

- 删掉逐项常量，直接用一个函数返回默认参数，按字段名赋值。字段顺序错了会编译不过，注释也只写一遍：

```cpp
constexpr AutoMoveTune MakeOdometryDefault()
{
    AutoMoveTune t = {};
    t.maxVelocity = 1.0f;        // m/s
    t.acceleration = 0.6f;       // m/s^2
    // ...
    return t;
}
```

（本工程是 C++11，`constexpr` 函数体只能有一条 `return` 语句。可以把语言标准升到 C++14 来用这种写法；不升级的话，就用普通函数，在 `tune_params.cpp` 初始化时调用。）

- 18 个字段的参数结构体只保留一份。可以保留度制的 `AutoMoveTune` 作为唯一定义，方便在 Watch 里直接改；`AutoMove::Start()` 收到参数时，在内部把 4 个角度字段换算成弧度。这样 `AutoMoveParam`、`ToAutoMoveParam()` 和那条 `static_assert` 都可以删掉。
- `robot_config.h:61-97` 的舵向 PID 是同样的写法，按同样方式处理。

### 10. 临界区代码重复 8 处

`bsp_usb.cpp`（4 处）、`dji_6020.cpp`（2 处）、`remote_input.cpp`、`vesc_motor.cpp` 都手写了 `__get_PRIMASK()` / `__disable_irq()` / `__set_PRIMASK()`。建议新增 `bsp/bsp_critical.h`：

```cpp
class CriticalSection
{
public:
    CriticalSection() : primask_(__get_PRIMASK()) { __disable_irq(); }
    ~CriticalSection() { __set_PRIMASK(primask_); }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t primask_;
};
```

使用时写 `{ CriticalSection lock; copy = state_; }`。即使中途 `return`，也不会忘记恢复中断。

### 11. 函数过长

| 函数 | 行数 | 拆分建议 |
|---|---|---|
| `AutoMove::Update()`（`auto_move.cpp:112`） | 约 128 | 拆成 `ComputeErrors()`、`ComputeCorrection()`、`ApplySettlingDeadband()` 三步，`Update()` 只负责串联 |
| `StartChassisTask()`（`Chassis_Task.cpp:60`） | 约 101 | 见第 12 条 |
| `Dji6020Bus::Control()`（`dji_6020.cpp`） | 约 87 | 两组发送逻辑重复，提取成 `SendGroup(groupIndex)` |
| `BspCan::Init()`（`bsp_can.cpp:20`） | 约 61 | FDCAN1 和 FDCAN3 两段几乎相同，提取成 `ConfigureBus(handle, idType, fifo, line)` |
| `VescMotor::ParseStatusPayload()` | 约 66 | 每种 STATUS 一个解析函数 |
| `Dji6020Bus::ParseFeedback()` | 约 58 | 拆出 ID 校验和字段解码 |

### 12. 任务函数承担了太多职责

`StartChassisTask()` 同时负责对象所有权（6 个函数内 `static` 对象）、调试登记、初始化顺序、模式状态机、调参应用和调试快照节流。建议新建 `ChassisApp` 类：

```cpp
class ChassisApp
{
public:
    void Init();
    void RunOnce(float dt);   // 一拍完整控制

private:
    ChassisMode SelectMode(const RemoteState& remote) const;
    void ApplyMode(ChassisMode mode, ...);

    Chassis chassis_;
    RemoteInput remote_;
    // ...
};
```

任务函数只剩下"创建、初始化、按固定周期调用 `RunOnce()`"，`g_debug.obj` 也只需要一个指针 `g_debug.app`。

### 13. 魔鬼数字

- `Chassis_Task.cpp:52-54`：拨杆值 `1U/3U/2U`，靠行尾注释说明含义。建议在 `remote_input.h` 定义 `enum class RemoteSwitch : uint8_t { Up = 1, Down = 2, Middle = 3 };`。
- `bsp_can.cpp:35,64` 以及协议层的 `data[8]`：定义 `constexpr uint8_t CAN_CLASSIC_PAYLOAD_SIZE = 8U;`。

### 14. 未使用的代码

| 位置 | 内容 |
|---|---|
| `chassis/Chassis.cpp:38` | `Chassis::Update()` 没有调用者，任务里分别调用的是 `UpdateFeedback()` 和 `UpdateControl()` |
| `chassis_kinematics.h:49-57` | `BodyVelocity()`、`Command()` 没有调用者 |
| `trapezoidal_profile.h:62-63` | `PeakVelocity()`、`TotalTime()` 没有调用者 |
| `pose_source.h:26` | `LidarOrigin()` 没有调用者 |
| `bsp_dwt.h` | `GetDeltaT64()`、`GetTimeline*()`、`DelayS()`、`sysTime_` 没有调用者 |
| `vesc_motor.h:85-97` | `SetPwm()`、`SetPos()`、`SetBrakeCurrent()`、`SetHandbrake*()` 没有调用者 |

驱动层的完整 API（VESC 的各种控制模式）可以保留，因为它描述的是设备能力。业务层和算法层中没有调用者的函数应删除。

### 15. 默认值不一致

`PoseFeedback::source` 的默认值是 `PoseSource::Lidar`（`motion_types.h:37`），其他 4 处默认都是 `Odometry`。建议增加 `PoseSource::None` 作为默认值，或者统一成同一个默认值。

## P2：命名、格式与注释

### 16. 文件命名

`Task/Chassis_Task.*`、`chassis/Chassis.*`、`debug/Debug_Snapshot.*` 改成全小写（`task/chassis_task.*`、`chassis/chassis.*`、`debug/debug_snapshot.*`）。改名时 Keil 和 EIDE 两个工程文件要同步修改，并单独提交。

### 17. 标识符命名

| 位置 | 现状 | 建议 |
|---|---|---|
| `vesc_motor.h:16` | `enum class CanPacketID` 的枚举值全大写（`SET_DUTY`） | 改成 `VescCommand::SetDuty`，和 `ChassisMode::Manual` 风格一致 |
| `dji_6020.h:20` | 命名空间 `Dji6020Cfg` 用缩写 | 改成 `Dji6020Protocol`，或者保留但统一使用 `Cfg` 后缀 |
| `bsp_dwt.h:32,39` | 参数 `cpu_freq_mhz`、`cnt_last` 使用 snake_case | 改成 `cpuFreqMhz`、`lastCount` |
| `bsp_can.cpp:22` | 局部变量 `FilterConfig` 首字母大写 | 改成 `filterConfig` |
| `vesc_motor.h:80` | `current_mA` | 改成 `currentMa` |

### 18. 格式

- 仓库里只有 `MDK-ARM/.clang-format`，配置是 Microsoft 基础风格、Linux 大括号，并且允许单行 `if`。这和实际代码的 Allman 风格不一致，说明它没有被使用。建议删掉，在仓库根目录放规范 4.2 节的配置。
- 超过 120 列的行有 29 处，集中在 `auto_move.cpp`、`auto_move_config.h`、`manual_mode_controller.cpp`。
- `auto_route_config.h:30-31` 混入了 Tab。
- `bsp_can.cpp` 有 10 处单行 `if ... return;` / `Error_Handler();`，第 110、140 行又混用了 K&R 大括号。

格式问题建议用 clang-format 一次性处理，作为单独的提交。

### 19. 注释

- 大部分文件没有文件头注释，建议按规范 2.5 节补上。
- 12 处注释符后面缺空格，并且 `//` 和 `/* */` 混用，例如 `trapezoidal_profile.h:10,23,69`、`auto_move.cpp:17,111,286,288`。
- 注释里记录了修改过程：`manual_mode_controller.cpp:14`（"沿用整改前的映射"）、`auto_move_config.h:36`（"数值从整改前 robot_config.h 原样保留"）。这类信息应该放进提交说明。
- `auto_move.cpp:233-234` 在两行代码之间插了两行长注释，复述上面已经做过的事。应删掉，或者移到函数头注释。
- `bsp_dwt.h:26` 提到的 `getTimeline_*` 是旧函数名，现在叫 `GetTimelineS()` 等。
- `auto_route_config.h:32-38` 是注释掉的航点。建议定义多个命名路线（`ROUTE_SQUARE`、`ROUTE_LINE_TEST`），用一个常量选择当前路线。

## P3：工程化

| # | 问题 | 建议 |
|---|---|---|
| 20 | `project_structure.md:34` 写了 `tests/` 目录，但仓库中不存在 | 新建主机测试，或者先把这一行删掉。完成第 6 条解耦后，PID、运动学、梯形规划、雷达协议都可以在 PC 上测试 |
| 21 | `MDK-ARM/.pack/` 下有 381 个文件被 git 跟踪 | 这是 Keil 器件包。如果希望别人克隆后能直接编译，可以保留；否则加入 `.gitignore`，在 README 中写明需要的 DFP 版本 |
| 22 | 没有 `.gitattributes` | 固定行尾格式，避免编辑器来回转换 CRLF 和 LF，导致 diff 里出现整个文件的改动 |
| 23 | 编译告警级别 | 确认 Keil 选的是 All Warnings，EIDE 加上 `-Wall -Wextra`，两边保持一致 |

## 关于你提供的 Template_Code.c

这份示范值得借鉴的地方：

- 文件头写明了职责和处理流程，读者能先看懂整体结构。
- 函数注释带 `@param[in]`、`@param[out]`、`@retval`，每个错误码都有说明。
- 模块内部函数全部 `static`，对外接口统一使用 `spk_` 前缀。
- 用 `1, 2, 3` 步骤注释组织长函数。

但它本身也不是一份合格的企业级代码，以下这些问题按任何一份规范都过不了评审：

- 有 20 处 `#if 0` 和大段注释掉的代码，`spk_demo()` 这样的演示代码也留在正式文件里。
- 有未使用的变量（`int i,j,k;`、`temp`、`event_temp`）。
- 用 `malloc()` 申请事件数组。嵌入式运行期应该静态分配。
- 注释和代码对不上：`spk_array_init()` 的 `@brief` 写的是"displaying"，是从别的函数复制过来没改。
- 有拼写错误：`vaild`、`voive`、`allcated`、`unvaild`。
- 线程函数长达 214 行。
- 包含顺序混乱，标准库头文件和 SDK 头文件交错排列。

结论：借鉴它的文档结构，不要照搬它的代码习惯。你现在的代码在命名、错误处理和函数长度上，已经比这份示范好。

## 建议的整改顺序

每一步单独提交，提交前用 Keil 和 EIDE 各编译一次，然后架空测试。

1. **P0 修复**：第 1-5 条，改动小，风险低。
2. **工具基线**：加 `.clang-format`、`.gitattributes`，全工程格式化。单独提交，提交内容只有格式变化。
3. **去重**：第 8、10、13、14 条，行为不变。
4. **解耦**：第 6、7 条。这一步改动 PID 和运动学接口，需要上车验证舵向控制。
5. **参数结构合并**：第 9 条。这一步涉及调参区，需要重新验证 `g_autoMoveTune`。
6. **拆分长函数和 `ChassisApp`**：第 11、12 条。
7. **改名**：第 16、17 条，同步修改两个工程文件。
8. **补测试**：第 20 条。
