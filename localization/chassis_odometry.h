#pragma once

#include "motion_types.h"

/**
 * @brief 用实测车体速度累计相对位姿，IMU 在线时使用其航向增量。
 *
 * IMU 离线时使用轮速解算的角速度；轮速反馈中断后需 Reset() 重新建原点。
 * 本类不读取传感器，也不参与电机控制。
 */
class ChassisOdometry
{
public:
    void Reset();

    /**
     * @param measuredBodyVelocity 轮速解算的车体速度，平移为 m/s，角速度为 rad/s。
     * @param imuYawDeg IMU 航向 (deg)，范围 -180~180。
     * @param imuOnline IMU 是否在线；离线或航向无效时使用轮速角速度。
     * @param dt 本次与上次更新的实际间隔 (s)。
     * @param wheelFeedbackValid 轮速、舵角反馈是否有效。
     */
    void Update(const ChassisBodyVelocity &measuredBodyVelocity,
                float imuYawDeg, bool imuOnline, float dt, bool wheelFeedbackValid);

    const ChassisPose2D &Pose() const { return pose_; }
    bool Valid() const { return valid_; }

private:
    ChassisPose2D pose_ = {};
    float lastImuYawDeg_ = 0.0f;
    bool imuYawInitialized_ = false;
    bool initialized_ = false;
    bool valid_ = false;
    bool feedbackLost_ = false;
};
