#pragma once

/**
 * @brief 相对起点的底盘位姿。
 * @note  x/y 单位 m，yaw 单位 rad；+x 向前，+y 向左，逆时针为正。
 */
struct ChassisPose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

/**
 * @brief 用实测车体速度和 IMU 航向累计相对位姿。
 *
 * 首帧有效反馈建立原点；反馈中断后停止积分，需 Reset() 重新建原点。
 * 本类不读取传感器，也不参与电机控制。
 */
class ChassisOdometry
{
public:
    void Reset();

    /**
     * @param bodyVx 车体前进速度 (m/s)。
     * @param bodyVy 车体向左速度 (m/s)。
     * @param imuYawDeg IMU 航向 (deg)，范围 -180~180。
     * @param dt 本次与上次更新的实际间隔 (s)。
     * @param feedbackValid 轮速、舵角和 IMU 反馈均有效。
     */
    void Update(float bodyVx, float bodyVy, float imuYawDeg,
                float dt, bool feedbackValid);

    const ChassisPose2D &Pose() const { return pose_; }
    bool Valid() const { return valid_; }

private:
    ChassisPose2D pose_ = {};
    float lastImuYawDeg_ = 0.0f;
    bool initialized_ = false;
    bool valid_ = false;
    bool feedbackLost_ = false;
};
