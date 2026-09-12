/**
 *  【DJI 6020 CAN 协议层说明】
 * ============================================================
 *  本文件只负责 6020 的协议部分：报文格式、电流标定、反馈解析。
 *  CAN 硬件操作（滤波器配置、外设启动、中断注册、发送帧对象）
 *  全部在 bsp/bsp_can.cpp 中完成，与 VESC(FDCAN1) 保持同一结构。
 *
 *  ── 与 VESC 的关键差异 ──
 *    1. 6020 是标准帧（11 位 ID），VESC 是扩展帧（29 位）
 *    2. 6020 一帧控制 4 个电机，VESC 一帧只控制 1 个
 *    3. 6020 用大端序 int16，VESC 用大端序 int32
 *
 *  ── 接收（电机 → 主控，反馈帧）──
 *    每个电机有独立反馈 ID：
 *      0x205 = 1 号    0x206 = 2 号    0x207 = 3 号    0x208 = 4 号
 *    数据布局（8 字节，大端序）：
 *    ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *    │Byte0 │Byte1 │Byte2 │Byte3 │Byte4 │Byte5 │Byte6 │Byte7 │
 *    │ 机械角度 0~8191  │ 转速 (rpm)      │ 转矩电流        │温度  │
 *    │   uint16 BE      │ int16 BE        │ int16 BE        │uint8 │
 *    └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *
 *  ── 发送（主控 → 电机，控制帧）──
 *    四个电机共用一帧，靠 ID 区分组别：
 *      0x1FF → 控制 1~4 号（反馈 0x205~0x208）
 *      0x2FF → 控制 5~8 号（反馈 0x209~0x20B）
 *    数据布局：4 个 int16 大端序，依次对应组内第 1~4 个电机
 *
 *  ── 电流标定 ──
 *    协议原始值 ±25000 对应 ±20 A（电调内部"转矩电流"，非电池总线电流）
 *    换算：raw = current_A × 1250        （25000 / 20 = 1250）
 * ============================================================
 */

#pragma once

#include "main.h"

/* ============================================================
 *  协议常量
 * ============================================================ */
namespace Dji6020Cfg
{
    /* 说明：这里用 const 而不是 constexpr。
     * 命名空间作用域的 const 整型/浮点常量同样是常量表达式（可作数组维度），
     * 语义与 constexpr 等价，但不要求编译器开启 C++11。
     * EIDE 的 C++ 标准是"按文件"配置的（.eide/files.options.yml），
     * 新增 .cpp 文件容易漏配 --cpp11，用 const 可以避免被这种配置问题卡住。 */

    /* --- CAN 标识符 --- */
    const uint32_t CTRL_ID_GROUP1   = 0x1FF;   ///< 控制 1~4 号电机
    const uint32_t CTRL_ID_GROUP2   = 0x2FF;   ///< 控制 5~8 号电机
    const uint32_t FEEDBACK_ID_BASE = 0x205;   ///< 反馈 ID 基数，电机 n 为 BASE + n - 1

    const uint32_t MOTOR_ID_MIN     = 1;       ///< 可配置的电机 ID 范围
    const uint32_t MOTOR_ID_MAX     = 8;

    /* --- 电流标定 --- */
    const int32_t  RAW_MAX          = 25000;   ///< 协议原始值上限，对应 +20 A
    const float    CURRENT_MAX_A    = 20.0f;   ///< 满量程电流 (A)
    const float    RAW_PER_AMP      = 1250.0f; ///< 每安培对应原始值 = 25000 / 20

    /* --- 反馈解析 --- */
    const float    ENCODER_MAX      = 8192.0f; ///< 机械角度分辨率
    const float    DEG_PER_TICK     = 360.0f / ENCODER_MAX;

    /* --- 本工程挂载数量与超时 --- */
    const uint32_t MOTOR_MAX        = 3;       ///< 实际挂载的 6020 数量
    const uint32_t ONLINE_TIMEOUT_MS = 100;    ///< 超时未收到反馈判定掉线
}

/* ============================================================
 *  单个电机的反馈数据（由 CAN 中断解析后填充）
 *
 *  读取方拿到的只是普通数值字段，中断里整体写入不会撕裂。
 * ============================================================ */
struct Dji6020RxData
{
    int16_t  encoder;        ///< 机械角度原始值 0~8191
    float    angleDeg;       ///< 单圈机械角度 0~360°
    float    totalAngleDeg;  ///< 累计多圈角度（跨零点自动处理，可正可负）
    int16_t  speedRpm;       ///< 转速 (rpm)，正负表示方向
    int16_t  currentRaw;     ///< 转矩电流原始值（±25000）
    float    currentAmp;     ///< 转矩电流 (A) = currentRaw / 1250
    uint8_t  temperature;    ///< 电机温度 (°C)

    bool     online;         ///< 是否在线（收到过反馈且在超时时间内）
    uint32_t lastRxTick;     ///< 最近一次收到反馈的系统时刻 (ms)
    uint32_t rxCount;        ///< 累计收到帧数（调试用，可判断丢帧）

    /* 内部状态：累计角度换算用，外部只读 */
    int16_t  lastEncoder;    ///< 上一帧角度
    bool     hasEncoder;     ///< 是否已收到过第一帧

    /* 全部清零。用构造函数而非 memset，避免破坏 C++ 对象语义 */
    Dji6020RxData()
        : encoder(0), angleDeg(0.f), totalAngleDeg(0.f)
        , speedRpm(0), currentRaw(0), currentAmp(0.f), temperature(0)
        , online(false), lastRxTick(0), rxCount(0)
        , lastEncoder(0), hasEncoder(false)
    {}
};

/* ============================================================
 *  Dji6020Bus —— 协议级总线操作（静态类）
 *
 *  【为什么发送在总线级而不是电机级】
 *  6020 的控制帧是"一帧控 4 个电机"的广播帧，四个电流值必须同时
 *  出现在同一帧的 8 字节里。单个电机对象无法完成发送 —— 它不知道
 *  其他电机的电流。所以：各电机写自己的槽，总线统一打包发送。
 *
 *  【调用时序】
 *  ① bsp 层：  BSP_CAN::Init()        —— 配滤波器、启动 FDCAN3（在 bsp_can.cpp）
 *  ② 控制：    Dji6020Bus::Control()  —— 周期任务中调用，打包并下发
 *  ③ 接收：    Dji6020Bus::ParseFeedback()—— 由 bsp_can 取帧后转发进来
 * ============================================================ */
class Dji6020Bus
{
public:
    /* --------------------------------------------------------
     *  把所有电机目标电流打包成控制帧并通过 BSP_CAN 下发
     *  应在周期任务中按固定频率调用（建议 1 kHz，与反馈帧率对齐）
     * -------------------------------------------------------- */
    static void Control(void);

    /* --------------------------------------------------------
     *  协议解析入口。由 bsp_can 取出报文后调用，本层不做任何 HAL 操作。
     *
     *  @param identifier  11 位标准帧 ID
     *  @param data        8 字节负载
     * -------------------------------------------------------- */
    static void ParseFeedback(uint32_t identifier, const uint8_t data[8]);

    /* --------------------------------------------------------
     *  超时判定：更新各电机在线状态
     *  （Control() 内已自动调用，此处单独暴露便于调试）
     * -------------------------------------------------------- */
    static void UpdateOnlineState(void);

private:
    /* 各电机在本帧中的电流槽 */
    struct ControlSlot
    {
        int16_t currentRaw;   ///< 原始电流值 ±25000
        bool    enabled;      ///< 是否参与控制（未 init 的电机不写入）
    };

    static ControlSlot ctrlSlot_[Dji6020Cfg::MOTOR_ID_MAX];
    static bool        hasMotor_;   ///< 是否已有电机注册（决定 Control() 是否发包）

    /* 仅供 Dji6020Motor 访问控制槽 */
    friend class Dji6020Motor;
};

/* ============================================================
 *  Dji6020Motor —— 单个 6020 电机对象
 *
 *  【使用方法】
 *  ① 绑定 ID（数组下标按机械位置排，与 CAN ID 顺序无关）：
 *       Dji6020Motors[0].init(1);   // 前轮 → 反馈 0x205
 *       Dji6020Motors[1].init(4);   // 左轮 → 反馈 0x208
 *       Dji6020Motors[2].init(3);   // 右轮 → 反馈 0x207
 *  ② 下发电流：
 *       Dji6020Motors[0].setCurrent(3.0f);   // 3 A
 *  ③ 读取反馈：
 *       float rpm = Dji6020Motors[0].getRxData().speedRpm;
 *  ④ 周期下发：
 *       Dji6020Bus::Control();
 * ============================================================ */
class Dji6020Motor
{
public:
    Dji6020Motor();

    /* --------------------------------------------------------
     *  绑定电机 ID（1~8），必须与电调实际配置一致
     *  1~4 由 0x1FF 控制，5~8 由 0x2FF 控制
     *
     *  ID 超范围会被拒绝并保持未初始化 —— 配错 ID 会导致电机
     *  完全不响应且没有任何报错，在初始化阶段拒绝能让问题早暴露。
     * -------------------------------------------------------- */
    void init(uint32_t motorId);

    /* --------------------------------------------------------
     *  设置目标转矩电流 (A)，正=正转、负=反转
     *  超出 ±20 A 自动限幅
     *
     *  只更新目标值，实际发送由 Dji6020Bus::Control() 完成 ——
     *  这样多个电机的电流才能被打进同一帧。
     * -------------------------------------------------------- */
    void setCurrent(float currentA);

    /* 按原始值设置（调试用，±25000 量程） */
    void setCurrentRaw(int16_t raw);

    /* 停止输出：目标电流置 0（零力矩，非刹车锁死） */
    void stop(void);

    /* 只读反馈数据 */
    const Dji6020RxData& getRxData(void) const { return rxData_; }

    /* 当前电机 ID（0 表示尚未初始化） */
    uint32_t getMotorId(void) const { return motorId_; }

private:
    uint32_t     motorId_;    ///< 电机标号 1~8，0 表示未初始化
    int32_t      slotIndex_;  ///< 在 ctrlSlot_ 中的下标，-1 表示未注册
    Dji6020RxData rxData_;    ///< 反馈数据

    /* 编码器增量法累计多圈角度 */
    void updateAngle(void);

    friend class Dji6020Bus;
};

/* ============================================================
 *  全局电机数组（下标 0~2 对应三路 6020）
 *  使用前必须各自调用 init() 绑定 CAN ID。
 *
 *  下标由机械位置决定，与 CAN ID 顺序无关：
 *      [0] 前轮 → 1 号（反馈 0x205）
 *      [1] 左轮 → 4 号（反馈 0x208）
 *      [2] 右轮 → 3 号（反馈 0x207）
 *  反馈分发按 ID 匹配查找，所以无论按什么顺序 init 都能正确对上。
 * ============================================================ */
extern Dji6020Motor Dji6020Motors[Dji6020Cfg::MOTOR_MAX];
