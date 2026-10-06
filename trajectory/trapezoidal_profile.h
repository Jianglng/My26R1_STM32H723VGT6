#pragma once

#include <stdint.h>

/** @brief 一维运动规划的当前阶段。 */
enum class TrapezoidalPhase : uint8_t
{
    Idle,
    Accelerating,
    Cruising,//匀速
    Decelerating,
    Finished
};

/**
 * @brief 当前参考输出，可在调试器中展开观察。
 * @note 位移单位 m，速度单位 m/s，时间单位 s。
 */
struct TrapezoidalProfileState
{
    float position = 0.0f;
    float velocity = 0.0f;
    float elapsedTime = 0.0f;//已流逝时间
    TrapezoidalPhase phase = TrapezoidalPhase::Idle;
};

/**
 * @brief 从静止出发、以零速度结束的一维梯形运动规划器。
 *
 * 距离不足以达到最大速度时，自动生成三角形曲线。
 * 只生成沿路径的参考位移和速度，不读取里程计，也不控制电机。
 */
class TrapezoidalProfile
{
public:
    /** @brief 取消当前规划，清零参数和输出，回到 Idle。 */
    void Reset();

    /**
     * @brief 启动或重新启动一段运动，每段运动只调用一次。
     * @param distance 路径长度 (m)，必须非负；方向由调用者处理。
     * @param maxVelocity 最大速度 (m/s)，必须大于零。
     * @param acceleration 加速度 (m/s^2)，必须大于零。
     * @param deceleration 减速度大小 (m/s^2)，必须大于零。
     * @return 参数及计算结果有效时返回 true；否则清零并回到 Idle。
     * @note 所有参数必须有限；零距离直接 Finished。
     */
    bool Start(float distance, float maxVelocity,
               float acceleration, float deceleration);

    /**
     * @brief 按实际时间间隔推进规划，不包含延时。
     * @param dt 距上次更新的实际间隔 (s)，必须有限且大于零。
     * @return Idle 或 dt 无效时返回 false，并保持当前输出。
     * @note Finished 后保持终点和零速度；大 dt 可以跨越多个阶段。
     */
    bool Update(float dt);

    const TrapezoidalProfileState &State() const { return state_; }
    float Position() const { return state_.position; }
    float Velocity() const { return state_.velocity; }
    float PeakVelocity() const { return peakVelocity_; }
    float TotalTime() const { return totalTime_; }

    /** @brief 参考轨迹是否结束，不代表底盘已实际到达目标点。 */
    bool Finished() const { return state_.phase == TrapezoidalPhase::Finished; }

private:
    TrapezoidalProfileState state_ = {};//当前的期望输出状态
    float distance_ = 0.0f;
    float acceleration_ = 0.0f;
    float deceleration_ = 0.0f;
    float peakVelocity_ = 0.0f;
    float accelTime_ = 0.0f;
    float cruiseTime_ = 0.0f;
    float totalTime_ = 0.0f;
    float accelDistance_ = 0.0f;
    float cruiseDistance_ = 0.0f;
};
