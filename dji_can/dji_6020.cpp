/**
 * @brief DJI 6020 协议层实现。
 *
 * 这里做电流标定、控制帧打包、反馈解析和累计角度。
 * 滤波器、启动、中断和发送帧在 bsp_can，本文件只把 8 字节交给它发出去。
 */

#include "dji_6020.h"

#include "bsp_can.h"

Dji6020Bus::ControlSlot Dji6020Bus::ctrlSlot_[Dji6020Cfg::MOTOR_ID_MAX];
bool                    Dji6020Bus::hasMotor_ = false;

/* 已注册的电机。下标是机械位置，不是 CAN ID。 */
Dji6020Motor* Dji6020Bus::motors_[RobotConfig::WHEEL_COUNT] = {};
uint32_t Dji6020Bus::motorCount_ = 0;

/**
 * @brief 把已注册电机的目标电流打包成控制帧，交给 BspCan 发送。
 * @note   建议 1 kHz 调用，与 6020 的反馈帧率对齐。
 *
 *         注意"写 0"和"不写入"在电调看来是两回事：停止下发会让电调
 *         进入失联保护，而写 0 是明确的零力矩指令。所以只要电机
 *         Init() 过，就应该一直留在控制帧里。
 */
void Dji6020Bus::Control()
{
    if (!hasMotor_)
    {
        return;
    }

    /* 总线异常先恢复，避免持续发送无效帧把 TX FIFO 塞满
     * （FIFO 只有 8 深，满了之后 HAL 会返回错误并丢帧） */
    BspCan::CheckBusOff(&hfdcan3);

    /* --- 收集各电机电流 ---
     * 控制槽按电机 ID 排列：下标 0~3 对应 0x1FF，下标 4~7 对应
     * 0x2FF。用 8 个 int32_t 临时量，避免把 5~8 号槽写出数组边界。
     */
    int32_t current[Dji6020Cfg::MOTOR_ID_MAX] = {0};
    bool group1Enabled = false;
    bool group2Enabled = false;

    for (uint32_t i = 0; i < Dji6020Cfg::MOTOR_ID_MAX; ++i)
    {
        if (ctrlSlot_[i].enabled)
        {
            current[i] = ctrlSlot_[i].currentRaw;

            if (i < 4U)
            {
                group1Enabled = true;
            }
            else
            {
                group2Enabled = true;
            }
        }
    }

    /* --- 限幅到协议量程 ±25000 --- */
    for (uint32_t i = 0; i < Dji6020Cfg::MOTOR_ID_MAX; ++i)
    {
        if (current[i] > Dji6020Cfg::RAW_MAX)
        {
            current[i] = Dji6020Cfg::RAW_MAX;
        }
        else if (current[i] < -Dji6020Cfg::RAW_MAX)
        {
            current[i] = -Dji6020Cfg::RAW_MAX;
        }
    }

    /* --- 打包并发送：每帧 4 个 int16，大端序 ---
     * 只有对应组存在已初始化电机时才发送该组，避免无意义的报文。
     */
    uint8_t data[8] = {0};

    if (group1Enabled)
    {
        for (uint32_t i = 0; i < 4U; ++i)
        {
            data[i * 2U]     = static_cast<uint8_t>((current[i] >> 8) & 0xFF);
            data[i * 2U + 1] = static_cast<uint8_t>(current[i] & 0xFF);
        }

        BspCan::fdcan3TxFrame_.header.Identifier = Dji6020Cfg::CTRL_ID_GROUP1;
        for (uint32_t i = 0; i < 8U; ++i)
        {
            BspCan::fdcan3TxFrame_.data[i] = data[i];
        }
        BspCan::AddMessageToTxFifoQ(&BspCan::fdcan3TxFrame_);
    }

    if (group2Enabled)
    {
        for (uint32_t i = 0; i < 4U; ++i)
        {
            data[i * 2U]     = static_cast<uint8_t>((current[i + 4U] >> 8) & 0xFF);
            data[i * 2U + 1] = static_cast<uint8_t>(current[i + 4U] & 0xFF);
        }

        BspCan::fdcan3TxFrame_.header.Identifier = Dji6020Cfg::CTRL_ID_GROUP2;
        for (uint32_t i = 0; i < 8U; ++i)
        {
            BspCan::fdcan3TxFrame_.data[i] = data[i];
        }
        BspCan::AddMessageToTxFifoQ(&BspCan::fdcan3TxFrame_);
    }

    /* 顺带更新在线状态，调用方不必额外维护 */
    UpdateOnlineState();
}

/**
 * @brief  协议解析入口：把一帧反馈分发到对应电机
 * @param  identifier  11 位标准帧 ID
 * @param  data        8 字节负载
 */
void Dji6020Bus::ParseFeedback(uint32_t identifier, const uint8_t data[8])
{
    /* 反馈 ID 从 0x205 开始；超出本工程电机数量的 ID 会在下面匹配失败 */
    if (data == nullptr || identifier < Dji6020Cfg::FEEDBACK_ID_BASE)
    {
        return;
    }

    /* 反馈 ID 反推电机标号：0x205 → 1 号电机 */
    const uint32_t motorId = identifier - Dji6020Cfg::FEEDBACK_ID_BASE + 1;

    if (motorId > Dji6020Cfg::MOTOR_ID_MAX)
    {
        return;
    }

    /* 按 motorId_ 匹配查找，不能用 ID 直接算下标 ——
     * 电机对象的下标由调用者按机械位置决定（前轮/左轮/右轮），
     * 与 CAN ID 顺序无关。本工程三个电机配的是 1/4/3 号，
     * 下标与 ID 不成线性关系，必须匹配。
     * 只有 3 个电机，循环开销可忽略，换来的是"改 ID 不用改代码"。
     */
    for (uint32_t i = 0; i < motorCount_; ++i)
    {
        Dji6020Motor &motor = *motors_[i];

        if (motor.motorId_ != motorId)
        {
            continue;   // 不是这个电机的帧
        }

        Dji6020RxData &rx = motor.rxData_;

        /* --- 解析（大端序，高字节在前）--- */
        rx.temperature = data[6];

        rx.encoder = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                           static_cast<uint16_t>(data[1]));

        /* 转速必须用有符号解析：反转时是负数，用 uint16_t 接会变成 60000+ */
        rx.speedRpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                                            static_cast<uint16_t>(data[3]));

        rx.currentRaw = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                                              static_cast<uint16_t>(data[5]));

        rx.currentAmp = static_cast<float>(rx.currentRaw) / Dji6020Cfg::RAW_PER_AMP;
        rx.angleDeg   = static_cast<float>(rx.encoder) * Dji6020Cfg::DEG_PER_TICK;

        /* 累计多圈角度 */
        motor.UpdateAngle();

        /* --- 在线状态 --- */
        rx.lastRxTick = HAL_GetTick();
        rx.online     = true;
        rx.rxCount++;

        return;   // 一个 ID 只对应一个电机，找到即结束
    }
}

/**
 * @brief  超时判定：超过 ONLINE_TIMEOUT_MS 没收到反馈视为掉线
 * @note   这是"通信健康度"而非"电机故障"——掉电、线松、波特率不匹配
 *         都会触发。控制层应据此做降级处理。
 *
 *         注意 lastRxTick 由中断写入、此处由任务读取，可能读到"上电时
 *         刻"（HAL_GetTick 很小）而非"从未收到"，所以额外用 lastRxTick==0
 *         判断"从未收到过"，避免刚上电就误报掉线。
 */
void Dji6020Bus::UpdateOnlineState()
{
    const uint32_t now = HAL_GetTick();
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint32_t i = 0; i < motorCount_; ++i)
    {
        Dji6020Motor &motor = *motors_[i];

        if (motor.motorId_ == 0 || motor.rxData_.lastRxTick == 0)
        {
            continue;
        }

        if ((now - motor.rxData_.lastRxTick) > Dji6020Cfg::ONLINE_TIMEOUT_MS)
        {
            motor.rxData_.online = false;
        }
    }

    __set_PRIMASK(primask);
}

Dji6020RxData Dji6020Motor::GetRxData() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Dji6020RxData copy = rxData_;
    __set_PRIMASK(primask);
    return copy;
}


void Dji6020Bus::Register(Dji6020Motor* motor)
{
    if (motor == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i < motorCount_; ++i)
    {
        if (motors_[i] == motor)
        {
            return;
        }
    }

    if (motorCount_ >= RobotConfig::WHEEL_COUNT)
    {
        return;
    }

    motors_[motorCount_++] = motor;
}

Dji6020Motor::Dji6020Motor()
    : motorId_(0)
    , slotIndex_(-1)
    , rxData_()
{
}

/**
 * @brief  绑定电机 ID 并注册到控制帧
 * @param  motorId  1~8，必须与电调实际配置一致
 */
void Dji6020Motor::Init(uint32_t motorId)
{
    /* 范围检查：ID 配错会导致电机完全不响应且无任何报错，
     * 所以在初始化阶段直接拒绝，让问题早暴露。 */
    if (motorId < Dji6020Cfg::MOTOR_ID_MIN || motorId > Dji6020Cfg::MOTOR_ID_MAX)
    {
        motorId_   = 0;
        slotIndex_ = -1;
        return;
    }

    motorId_   = motorId;
    slotIndex_ = static_cast<int32_t>(motorId - 1);   // ID 1 → 槽 0

    /* 注册到总线控制槽，初始电流 0 */
    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = 0;
    Dji6020Bus::ctrlSlot_[slotIndex_].enabled    = true;

    /* 第一个电机注册时标记协议层已就绪 */
    Dji6020Bus::hasMotor_ = true;
    Dji6020Bus::Register(this);
}

/**
 * @brief  设置目标电流 (A)，自动限幅到 ±20 A
 */
void Dji6020Motor::SetCurrent(float currentA)
{
    if (slotIndex_ < 0)
    {
        return;   // 未初始化
    }

    /* 先限幅再换算：避免 currentA 极大时乘法溢出 */
    if (currentA > Dji6020Cfg::CURRENT_MAX_A)
    {
        currentA = Dji6020Cfg::CURRENT_MAX_A;
    }
    else if (currentA < -Dji6020Cfg::CURRENT_MAX_A)
    {
        currentA = -Dji6020Cfg::CURRENT_MAX_A;
    }

    const int32_t raw = static_cast<int32_t>(currentA * Dji6020Cfg::RAW_PER_AMP);

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = static_cast<int16_t>(raw);
}

/**
 * @brief  按原始值设置（调试用）
 */
void Dji6020Motor::SetCurrentRaw(int16_t raw)
{
    if (slotIndex_ < 0)
    {
        return;
    }

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = raw;
}

/**
 * @brief  零力矩停止
 */
void Dji6020Motor::Stop()
{
    if (slotIndex_ < 0)
    {
        return;
    }

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = 0;
}

/**
 * @brief  编码器增量法累计多圈角度
 *
 *  6020 编码器是单圈绝对值型（0~8191），上电即知绝对位置，
 *  但跨零点时会从 8191 跳到 0。控制周期远快于机械转速，因此
 *  相邻两帧的真实增量一定小于半圈：
 *    增量 > 4096  → 实际是反向跨零（少算了一圈）
 *    增量 < -4096 → 实际是正向跨零（多算了一圈）
 */
void Dji6020Motor::UpdateAngle()
{
    Dji6020RxData &rx = rxData_;

    if (!rx.hasEncoder)
    {
        /* 第一帧：以当前位置为累计角度原点，避免上电瞬间跳一个随机角度 */
        rx.lastEncoder   = rx.encoder;
        rx.totalAngleDeg = 0.0f;
        rx.hasEncoder    = true;
        return;
    }

    int32_t delta = static_cast<int32_t>(rx.encoder) - static_cast<int32_t>(rx.lastEncoder);

    /* 跨零点修正：满量程 8192，半圈 4096 */
    const int32_t half = static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX / 2.0f);

    if (delta > half)
    {
        delta -= static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX);
    }
    else if (delta < -half)
    {
        delta += static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX);
    }

    rx.totalAngleDeg += static_cast<float>(delta) * Dji6020Cfg::DEG_PER_TICK;

    rx.lastEncoder = rx.encoder;
}
