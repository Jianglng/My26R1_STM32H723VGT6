# My26R1 嵌入式 C++ 编码规范

本规范适用于本工程的应用代码（`task/`、`chassis/`、`control/`、`localization/`、`algorithm/`、`protocol/`、`bsp/`、`common/`、`robot_config/`、`debug/`）。CubeMX 生成的 `Core/`、`USB_DEVICE/`、`Drivers/`、`Middlewares/` 以及第三方源码（如 `protocol/imu/hipnuc_*`）保持原样，不按本规范改写。

参考来源：Google C++ 风格指南（中文版）、浙大 Hello World 嵌入式编程风格指南、getiot C 语言编程规范（华为规范整理版），以及 MISRA C/C++ 的安全思路。各条规则的出处和本工程的取舍见文末附录。

规则分两级：

- **必须**：违反即视为缺陷，代码评审不通过。
- **建议**：没有特殊理由时应遵守；不遵守时要在代码里写明原因。

## 0. 总体原则

1. **清晰第一。** 代码首先给人读，其次才给机器执行。能写成直白代码的，不要写成技巧性代码。
2. **一致性优先于个人偏好。** 新代码要沿用本规范和周边代码的风格。不要在一个文件里混用两种风格。
3. **单一事实来源。** 同一个物理量、参数或常量只在一个地方定义，其它地方都引用它。
4. **安全优先。** 任何异常（通信丢失、参数非法、计算溢出）都必须导向停机或保持零输出，不能导向未知动作。

## 1. 工程结构与分层

### 1.1 分层与依赖方向（必须）

```text
task                    任务调度：拥有对象、决定调用顺序
  ↓
chassis / control / localization     业务层
  ↓
algorithm               纯算法：不碰硬件、不依赖具体车型
protocol                设备协议：组帧、解析
  ↓
bsp                     板级：HAL 封装、中断、收发缓存
  ↓
HAL / CMSIS / FreeRTOS
```

- 只能向下依赖，禁止反向依赖和循环依赖。
- `algorithm/` 禁止包含 `main.h`、HAL 头文件、`bsp_*.h` 和 `robot_config.h`。车型参数通过参数或构造函数传入，硬件计时通过接口或 `dt` 参数传入。
- `protocol/` 只负责报文格式。总线句柄、发送帧缓冲区归 `bsp/` 所有，`protocol/` 通过函数接口发送，禁止直接改 `bsp` 的成员变量。
- `common/` 只放跨层共享的纯数据类型，不放逻辑。
- `robot_config/` 只放数值和纯数据结构，不包含控制层头文件。
- `debug/` 只读业务对象，任何控制路径都不能读取 `debug/` 的数据。

### 1.2 模块组织（建议）

- 一个模块一个目录，一个类对应一对 `.h/.cpp`，文件名与类名对应（类 `ChassisOdometry` 对应 `chassis_odometry.h/.cpp`）。
- 头文件只暴露其他模块需要的接口；只在本文件使用的函数和变量放进 `.cpp` 的匿名命名空间。

## 2. 文件与头文件

### 2.1 文件命名（必须）

- 全部小写，单词之间用下划线：`chassis_task.cpp`、`debug_snapshot.h`。
- 目录名同样全部小写。
- 专有型号保持原写法：`dji_6020.cpp`、`vesc_motor.cpp`。
- C++ 源文件用 `.cpp`，头文件用 `.h`。

### 2.2 头文件保护（必须）

- 统一使用 `#pragma once`。AC6、GCC、Clang 都支持，本工程不考虑其他编译器。
- 第三方和 CubeMX 文件保留原来的 `#ifndef` 写法。

### 2.3 头文件内容（必须）

- 头文件必须自包含：单独 `#include` 它就能编译通过。
- 只包含用到的头文件。只用到指针或引用的类型，优先用前向声明。
- 头文件中禁止定义非 `const` 变量。全局变量在 `.cpp` 中定义，在头文件中 `extern` 声明。
- 禁止在 `.cpp` 中用 `extern` 声明别的模块的变量或函数，必须包含对方的头文件。CubeMX 句柄（`hfdcan1`、`huart5` 等）通过对应的 `fdcan.h`、`usart.h` 获取。
- 需要给 C 文件调用的接口用 `extern "C"` 包裹，且只包裹函数声明，不在 `extern "C"` 块里 `#include`。

### 2.4 包含顺序（必须）

```cpp
#include "chassis_odometry.h"     // 1. 本文件对应的头文件

#include <math.h>                 // 2. C/C++ 标准库
#include <stdint.h>

#include "cmsis_os.h"             // 3. HAL、CMSIS、FreeRTOS 等外部库
#include "main.h"

#include "math_utils.h"           // 4. 本工程其他模块
#include "motion_types.h"
```

每组之间空一行，组内按字母顺序排列。

### 2.5 文件头注释（必须）

每个 `.h/.cpp` 文件开头写明文件职责。版本历史交给 git，不在文件里维护修改日志。

```cpp
/**
 * @file    chassis_odometry.h
 * @brief   用实测车体速度和 IMU 航向累计底盘相对位姿
 * @note    不读取传感器，不参与电机控制；调用者保证 1 kHz 周期调用
 */
```

## 3. 命名

### 3.1 通用规则（必须）

- 名字要能说明含义，少用缩写。只允许业界通用缩写：`rx`、`tx`、`cnt`、`err`、`cfg`、`pid`、`imu`、`can`、`usb`、`rpm`、`dt`。
- 禁止拼音和无意义的单字母名。`i`、`j`、`k` 只用作循环变量。
- 带物理单位的量，名字里写单位：`timeoutMs`、`wheelDiameterM`、`yawDeg`、`wzDegPerSec`。弧度和国际单位制是默认值，可以不写。
- 布尔量用 `is/has/can/...` 或形容词：`online`、`hasFrame`、`feedbackValid`。
- 互斥的名字成对出现：`Start/Stop`、`Enable/Disable`、`Init/Deinit`。

### 3.2 命名风格

本工程已经形成了一套接近 Google 的风格。为避免全工程改名，沿用现有写法，并统一成下表：

| 类别 | 写法 | 示例 | 与 Google 的差别 |
|---|---|---|---|
| 类、结构体、枚举、类型别名 | 大驼峰 | `ChassisOdometry`、`PidParam` | 相同 |
| 函数、成员函数 | 大驼峰 | `UpdateFeedback()`、`GetRxData()` | 相同 |
| 局部变量、参数、结构体成员 | 小驼峰 | `measuredVelocity`、`dt` | Google 用 `snake_case` |
| 类私有成员 | 小驼峰加下划线后缀 | `steerAnglePid_` | 后缀相同 |
| 编译期常量（`constexpr`、`static const`） | 全大写下划线 | `WHEEL_COUNT`、`LIDAR_TIMEOUT_MS` | Google 用 `kWheelCount` |
| `enum class` 枚举值 | 大驼峰 | `ChassisMode::Manual` | Google 用 `kManual` |
| 命名空间 | 大驼峰 | `RobotConfig`、`MathUtils` | Google 用小写 |
| 宏 | 全大写下划线，加模块前缀 | `CHASSIS_DEBUG_ENABLE` | 相同 |
| 全局变量 | `g_` 前缀加小驼峰 | `g_debug`、`g_autoMoveTune` | 华为规范写法 |
| 文件、目录 | 全小写下划线 | `chassis_task.cpp` | 相同 |

补充规则：

- 优先使用 `enum class`。只有用作位掩码时才用普通 `enum`，此时枚举值要带类型名前缀。
- 私有静态成员同样加 `_` 后缀。
- 公有结构体成员不加 `_` 后缀。

## 4. 格式

格式交给工具处理。仓库根目录放一份 `.clang-format`，提交前格式化，评审时不再讨论格式。

### 4.1 基本规则（必须）

- 缩进 4 个空格，禁止 Tab。
- 每行不超过 120 列。
- 文件编码 UTF-8，行尾统一（建议用 `.gitattributes` 固定为 LF 或 CRLF 之一）。
- 大括号单独占一行（Allman 风格），与现有代码一致。
- `if`、`for`、`while`、`do` 的语句体必须用大括号，即使只有一行。
- 一行只写一条语句。
- 运算符两侧加空格，逗号后加空格，括号内侧不加空格。
- `switch` 必须有 `default`。一个 `case` 有意贯穿到下一个时，必须注释 `/* fallthrough */`。
- 空循环体写 `{}`，不要写单独的分号。

### 4.2 推荐的 `.clang-format`

```yaml
BasedOnStyle: Google
IndentWidth: 4
TabWidth: 4
UseTab: Never
ColumnLimit: 120
BreakBeforeBraces: Allman
AccessModifierOffset: -4
NamespaceIndentation: All
IndentCaseLabels: true
AllowShortIfStatementsOnASingleLine: Never
AllowShortLoopsOnASingleLine: false
AllowShortBlocksOnASingleLine: Never
AllowShortFunctionsOnASingleLine: InlineOnly
PointerAlignment: Left
DerivePointerAlignment: false
SortIncludes: false
IncludeBlocks: Preserve
FixNamespaceComments: true
SpacesBeforeTrailingComments: 2
```

## 5. 注释

### 5.1 原则（必须）

- 注释解释“为什么”和“约束条件”，不复述代码在做什么。好的命名能省掉大部分注释。
- 注释语言统一用中文，专有名词保留英文；中文代码注释不加句号，必要时分行或使用逗号、分号。
- 修改代码时同步修改周边注释。过时的注释比没有注释更糟。
- 注释放在代码上方或行尾，不放在下方。上方注释与前面的代码之间空一行。
- `//` 和 `/*` 后面空一格。
- 不在注释里记录修改过程（“整改前”“原样保留”“改为……”）。这些信息写进 git 提交说明。

### 5.2 函数注释（必须）

对外接口在头文件声明处写 Doxygen 注释，说明功能、参数、返回值和调用约束（线程或中断上下文、调用顺序、单位）：

```cpp
/**
 * @brief  按实际时间间隔推进一维梯形规划
 * @param  dt  距上次更新的时间间隔 (s)，由任务层保证有限且大于 0
 * @note   Idle 时保持输出，内部不重复校验 dt
 * @note   只在底盘任务中调用，不可重入
 */
void Update(float dt);
```

简单的取值函数（`Pose()`、`Valid()`）可以不写。

### 5.3 变量与成员注释（必须）

- 全局变量、模块级静态变量必须注释用途、取值范围、由谁写、由谁读。
- 有单位的结构体成员，在行尾注明单位。

### 5.4 TODO（必须）

临时方案统一写成 `// TODO(作者): 说明`，方便全局搜索。禁止无说明的 TODO。

## 6. 函数

### 6.1 职责（必须）

- 一个函数只做一件事。
- 非空非注释代码不超过 50 行，嵌套不超过 4 层。超过时拆分子函数。
- 参数不超过 5 个。超过时用结构体打包。
- 参数顺序：输入在前，输出在后。

### 6.2 参数与返回值（必须）

- 只读的大对象用 `const &` 传入；输出参数用非 `const` 引用；可以为空的参数才用指针。
- 优先用返回值输出结果。多个输出时返回结构体。
- 返回错误码或 `bool` 的函数，调用者必须处理返回值。不处理时显式写 `(void)` 并注释原因。
- 能声明为 `const` 的成员函数必须声明为 `const`。

### 6.3 参数检查（必须）

- 在真实输入边界检查一次：协议解码检查报文和数值，在线调参检查候选配置，任务层检查实际 `dt`。公共接口不等于外部输入边界，内部算法的公开接口也可由调用者保证输入有效。
- 内部算法不重复检查已经验证的位姿、实测速度、`dt` 和乘加中间结果；头文件须说明调用约束。送入底盘的速度，以及转为电流整数前的输出保留最终非有限值检查。
- `constexpr` 配置用 `static_assert` 验证，不在每次启动运动时执行运行期检查。
- 同一类校验只写一份，其他地方调用它（例如 `PidParamValid()`）。

## 7. 类与结构体

### 7.1 struct 与 class（必须）

- 只存数据、没有不变量的被动对象用 `struct`，成员公开。
- 有行为、有不变量的对象用 `class`，数据成员一律 `private`。
- 调试需要观察内部状态时，提供 `const` 访问函数（如 `const PidState& State() const`），不要为了调试把成员改成 `public`。Keil 调试器可以直接展开私有成员。

### 7.2 构造与初始化（必须）

- 全局或静态对象的构造函数不得调用 HAL，也不得依赖其他模块的初始化顺序。硬件初始化统一放在显式的 `Init()` 中。
- 所有成员必须有确定初值，优先用类内默认成员初始化（`float x = 0.0f;`）。
- 单参数构造函数加 `explicit`。
- 不可拷贝的对象（持有硬件资源、注册到总线的对象）显式 `= delete` 拷贝构造和拷贝赋值。

### 7.3 声明顺序（建议）

`public` → `protected` → `private`。每段内依次是：类型、常量、构造与析构、成员函数、数据成员。

### 7.4 继承（建议）

- 优先组合，少用继承。
- 重写虚函数必须写 `override`。
- 中断和 1 kHz 控制路径上避免虚函数调用。

## 8. 常量、宏与类型

### 8.1 常量（必须）

- 禁止魔鬼数字。除 `0`、`1`、`-1` 和显而易见的数学系数（如 `0.5f`、`2.0f`）外，数值都要定义成有名字的常量。
- 常量用 `constexpr` 定义，不用 `#define`。
- 通用数学常量（`PI`、度和弧度换算）只在 `math_utils.h` 定义一次。
- 车型相关的数值只放在 `robot_config/`。

### 8.2 宏（必须）

- 只在条件编译、`__FILE__`/`__LINE__` 这类必须由预处理器完成的场景用宏。能用 `constexpr` 或 `inline` 函数的地方都不用宏。
- 表达式宏的每个参数和整体都加括号；多语句宏用 `do { ... } while (0)` 包裹。
- 禁止在宏参数中写带副作用的表达式（如 `SQUARE(i++)`）。

### 8.3 类型（必须）

- 整数一律使用 `<stdint.h>` 定宽类型（`uint8_t`、`int32_t` 等）。长度和下标用 `uint32_t` 或 `size_t`。
- 浮点统一用 `float`，字面量写 `f` 后缀。Cortex-M7 的 FPU 只对单精度做硬件加速，`double` 会走软件库或双精度单元，控制路径上禁止使用。
- 无符号字面量写 `U` 后缀。
- 用 `static_cast<>` 做类型转换，禁止 C 风格强制转换。所有可能截断或改变符号的转换都要显式写出。
- 有符号数和无符号数不得直接比较。
- 指针空值用 `nullptr`，不用 `NULL` 或 `0`。
- 只在局部变量上用 `auto`，而且只在类型一眼可见时使用。

## 9. 嵌入式专项

### 9.1 内存（必须）

- 运行期禁止动态内存分配：不用 `malloc/free`、`new/delete`、`std::vector`、`std::string`。所有对象静态分配，或在任务启动时一次性分配。
- 禁止变长数组和 `alloca()`。
- 任务里的大对象（超过约 256 字节）不要放在栈上，改为 `static` 或成员变量。每个任务的栈使用量通过 `uxTaskGetStackHighWaterMark()` 定期核对。
- 通信帧使用固定长度缓冲区，写入前必须检查边界。

### 9.2 中断（必须）

- 中断服务函数和 HAL 回调只做最少的工作：取数据、写缓冲区、置标志。解析、计算、打印都放到任务里。
- 中断中禁止调用阻塞 API，FreeRTOS 只能调用 `...FromISR` 版本。
- 全工程每个 HAL 弱回调（如 `HAL_UARTEx_RxEventCallback`、`HAL_FDCAN_RxFifo0Callback`）只能在一个文件里实现，由 `bsp/` 统一分发。

### 9.3 共享数据与并发（必须）

- 中断与任务共享的发布标志、索引、计数器使用 `volatile`；多字段数据在临界区中取快照。普通缓冲区由唯一中断生产者写入，经内存屏障发布，任务在临界区内读取，不逐字节添加 `volatile` 或进行重复校验。
- 临界区要短，只包含拷贝操作，不在临界区内调用 HAL 或 FreeRTOS API。
- 统一使用一组临界区封装（例如 `bsp/bsp_critical.h` 中的 `CriticalSection` RAII 类），不在各模块里重复写 `__get_PRIMASK()` / `__disable_irq()` / `__set_PRIMASK()`。
- 多个任务共享的数据，用 FreeRTOS 队列或互斥量保护。明确每份数据的唯一写者。
- `volatile` 不能替代临界区，也不能保证多字段数据的一致性。

### 9.4 时间（必须）

- 时间戳用 `uint32_t`，比较时写成无符号减法 `(now - last) >= period`，这样计数回绕时也正确。
- 控制周期的 `dt` 由调用者实测后传入，算法模块不直接读取硬件计时器。
- 任务中禁止忙等延时，使用 `osDelay()` / `vTaskDelayUntil()`。固定周期的控制任务用 `vTaskDelayUntil()`，避免周期累积漂移。

### 9.5 外设与 HAL（必须）

- 检查所有 HAL 函数的返回值。初始化失败要进入明确的安全状态（停机并记录错误码），不能悄悄继续运行。
- 外设句柄只在 `bsp/` 中直接使用；上层通过 `bsp/` 的接口访问。

## 10. 错误处理与安全

### 10.1 失效安全（必须）

- 遥控离线、通信超时、反馈无效、参数非法、计算结果非有限值，都必须导向零输出。
- 每个外部输入源都要有在线判定和超时时间，超时时间定义成常量。
- 原则上启用独立看门狗（IWDG），在控制主循环中喂狗。主循环卡死时由硬件复位。当前工程按用户 2026-10-30 的明确要求暂缓添加 IWDG，本轮不配置、不喂狗。

### 10.2 断言（建议）

- 用断言检查编程错误（不可能出现的状态、违反调用约定），不用断言处理运行期错误（通信丢失、用户输入错误）。
- 编译期能检查的约束用 `static_assert`。

### 10.3 错误记录（建议）

- 统一一个错误码枚举，模块出错时记录错误码和计数，调试器中可以直接查看。

## 11. 调试与调参

- 调试观察和在线调参的变量集中放在 `debug/` 和 `robot_config/tune_params.*`，调试器入口尽量少。
- 调试代码不能改变控制行为。调试快照只读业务对象，低频更新。
- 禁止用 `#if 0` 或大段注释保留旧代码。删掉的代码在 git 历史里可以找回。
- 调试开关用一个统一的宏控制（如 `DEBUG_SNAPSHOT_ENABLE`），发布配置中关闭。

## 12. 构建与版本管理

### 12.1 编译（必须）

- 应用统一使用 C++14，默认参数通过具名字段赋值的 `constexpr` 构造函数定义，避免同类型聚合字段顺序出错。

- 打开编译器的高告警级别（AC6：`-Wall -Wextra`，Keil 中 Warnings 选 `All Warnings`），新代码零告警。
- 告警通过修改代码消除，不通过降低告警级别消除。
- Keil 工程和 EIDE 工程的编译选项、源文件列表保持一致。新增源文件时两边同时添加。

### 12.2 版本管理（必须）

- 构建产物、本地工具缓存不进仓库（`.o`、`.axf`、`.map`、`build/`、`.pack/` 等写进 `.gitignore`）。
- 一次提交只做一件事。功能修改、格式化、重命名分开提交。
- 提交说明写清楚改了什么、为什么改；调好的参数要在提交说明里写明实测条件。
- 每次提交前保证 Keil 和 EIDE 都能编译通过。

## 13. 测试

- `algorithm/`、`protocol/` 中的纯逻辑代码应能在 PC 上编译，并写主机单元测试（推荐 GoogleTest 或 doctest）。
- 修复缺陷时，先补一个能复现该缺陷的测试，再修改代码。
- 实车测试按固定步骤执行：先架空验证，再落地低速验证，最后满速验证。

## 附录 A：规则来源

| 本规范章节 | 主要参考 | 本工程的取舍 |
|---|---|---|
| 0 总体原则 | getiot 0.2 | 增加“单一事实来源”“安全优先” |
| 1 分层 | 浙大 2.5、getiot 原则 1.3 | 按本工程目录明确依赖方向 |
| 2 头文件 | Google 1、getiot 1 | 用 `#pragma once` 代替 `#ifndef` |
| 3 命名 | Google 7、浙大 6、getiot 3 | 沿用现有大小驼峰和全大写常量，不改成 `snake_case` 和 `kName` |
| 4 格式 | Google 8、浙大 8/9 | 4 空格、Allman，与现有代码一致 |
| 5 注释 | 浙大 7、getiot 8、示例模板 | 采用 Doxygen 格式；版本历史交给 git |
| 6 函数 | Google 4、getiot 2 | 50 行、4 层嵌套、5 个参数 |
| 7 类 | Google 3、浙大 3 | 禁止为调试公开数据成员 |
| 8 常量与类型 | getiot 5/10、浙大 5、MISRA | 强调 `float` 和定宽整数 |
| 9 嵌入式专项 | 浙大 2.5/4.4、getiot 6、MISRA | 本工程补充 |
| 10 安全 | getiot 6/12 | 本工程补充看门狗和零输出 |
| 12 构建 | getiot 11 | 补充 Keil/EIDE 双工程 |

## 附录 B：评审检查清单

提交前逐项确认：

- [ ] Keil 和 EIDE 都编译通过，没有新增告警
- [ ] 已用 `.clang-format` 格式化
- [ ] 没有新增魔鬼数字、重复常量、`#if 0`、注释掉的代码
- [ ] 新增的公有接口有 Doxygen 注释，带单位
- [ ] 没有为了调试把成员改成 `public`
- [ ] 中断与任务共享的数据有临界区保护
- [ ] 所有 HAL 返回值都已处理
- [ ] 异常路径都导向零输出
- [ ] 新增 `.cpp` 已同时加入 Keil 和 EIDE 工程
