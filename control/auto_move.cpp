#include "auto_move.h"

#include "math_utils.h"

#include <math.h>

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float MAX_UPDATE_INTERVAL_S = 0.05f;

    bool PoseValid(const ChassisPose2D &pose)
    {
        return isfinite(pose.x) && isfinite(pose.y) && isfinite(pose.yaw);
    }

    bool ParamValid(const AutoMoveParam &param)
    {
        return isfinite(param.maxVelocity) && param.maxVelocity > 0.0f &&
               isfinite(param.acceleration) && param.acceleration > 0.0f &&
               isfinite(param.deceleration) && param.deceleration > 0.0f &&
               isfinite(param.alongKp) && param.alongKp >= 0.0f &&
               isfinite(param.crossKp) && param.crossKp >= 0.0f &&
               isfinite(param.yawKp) && param.yawKp >= 0.0f &&
               isfinite(param.maxCorrectionVelocity) && param.maxCorrectionVelocity > 0.0f &&
               isfinite(param.maxOutputVelocity) && param.maxOutputVelocity >= param.maxVelocity &&
               isfinite(param.maxYawVelocity) && param.maxYawVelocity > 0.0f &&
               isfinite(param.yawAcceleration) && param.yawAcceleration > 0.0f &&
               isfinite(param.positionTolerance) && param.positionTolerance > 0.0f &&
               isfinite(param.yawTolerance) && param.yawTolerance > 0.0f && param.yawTolerance <= PI &&
               isfinite(param.speedTolerance) && param.speedTolerance > 0.0f &&
               isfinite(param.yawSpeedTolerance) && param.yawSpeedTolerance > 0.0f &&
               isfinite(param.settleDuration) && param.settleDuration > 0.0f;
    }

    void LimitVector(float &x, float &y, float limit)
    {
        const float length = hypotf(x, y);
        if (length > limit)
        {
            const float scale = limit / length;
            x *= scale;
            y *= scale;
        }
    }
}

void AutoMove::Reset()
{
    distancePlanner_.Reset();
    param_ = AutoMoveParam{};
    motionState_ = AutoMoveState{};
    pathLength_ = 0.0f;
    directionX_ = 1.0f;
    directionY_ = 0.0f;
}

bool AutoMove::Start(const ChassisPose2D &currentPose, const AutoMoveTarget &target, const AutoMoveParam &param)
{
    /* 清除上一段运动；每段只调用一次，底盘停稳和反馈有效由任务确认。 */
    Reset();

    /* 检查位姿、目标和控制参数，失败时进入故障并保持零输出。 */
    if (!PoseValid(currentPose) || !isfinite(target.x) || !isfinite(target.y) || !isfinite(target.yaw) || !ParamValid(param))
    {
        SetFault();
        return false;
    }

    /* 起点到目标点的世界坐标系位移；hypotf(dx, dy) 得到路径长度。 */
    const float dx = target.x - currentPose.x;
    const float dy = target.y - currentPose.y;
    const float length = hypotf(dx, dy);
    /* 用总长度启动一维梯形规划，运动方向由下方的单位向量处理。 */
    if (!isfinite(length) || !distancePlanner_.Start(length, param.maxVelocity, param.acceleration, param.deceleration))
    {
        SetFault();
        return false;
    }

    /* 保存本段参数、起点和目标；起点在后续 Update() 中保持不变。 */
    param_ = param;
    motionState_.startPose = currentPose;
    motionState_.target = target;

    motionState_.positionError = length;
    pathLength_ = length;

    if (pathLength_ > 0.0f)
    {
        /* 沿路径单位方向 u=(dx/L, dy/L)，用来映射参考位移和分解误差。 */
        directionX_ = dx / pathLength_;
        directionY_ = dy / pathLength_;
        motionState_.phase = AutoMovePhase::Tracking;
    }
    else
    {
        /* 零长度不求单位方向，沿用 Reset() 的世界 x/y 方向进行终点修正。
         * 此时位置已在目标点，也仍需修正航向并检查是否停稳。 */
        motionState_.phase = AutoMovePhase::Settling;
    }
    /* 参考位置从起点开始推进，参考航向从第一拍起就使用指定目标。 */
    UpdateReference();
    return true;
}

//梯形速度前馈+位置P反馈
void AutoMove::Update(const ChassisPose2D &currentPose, const ChassisBodyVelocity &measuredVelocity, float dt, bool feedbackValid)
{
    /* 待机不推进轨迹；故障保持零输出，需复位或重新启动才能恢复。 */
    if (motionState_.phase == AutoMovePhase::Idle || Faulted())
    {
        return;
    }

    /* 检查本拍反馈和实际 dt；数据异常或更新间隔过长时立即清零输出。 */
    if (!feedbackValid || !PoseValid(currentPose) ||
        !isfinite(measuredVelocity.vx) || !isfinite(measuredVelocity.vy) ||
        !isfinite(measuredVelocity.wz) ||
        !isfinite(dt) || dt <= 0.0f || dt > MAX_UPDATE_INTERVAL_S)
    {
        SetFault();
        return;
    }

    /* 完成后保持零输出，不因误差变化自行重新启动运动 */
    if (Finished())
    {
        return;
    }

    /*按 dt 推进参考位移和速度。规划结束只切到终点修正，不直接报完成 */
    if (motionState_.phase == AutoMovePhase::Tracking)
    {
        if (!distancePlanner_.Update(dt))
        {
            SetFault();
            return;
        }
        if (distancePlanner_.Finished())
        {
            motionState_.phase = AutoMovePhase::Settling;
        }
    }

    /* 将一维位移映射成世界参考位置；终点修正时固定为最终目标点。 */
    UpdateReference();

    /* 参考减实际得到位置误差，再投影到路径方向 u 和法向 n=(-uy, ux)
     * alongError 为正表示落后参考位置；crossError 的正方向是 n
     * 这里的横向垂直于路径，斜向运动时也随路径方向一起旋转 */
    const float errorX = motionState_.referencePose.x - currentPose.x;
    const float errorY = motionState_.referencePose.y - currentPose.y;
    motionState_.alongError = directionX_ * errorX + directionY_ * errorY;
    motionState_.crossError = -directionY_ * errorX + directionX_ * errorY;

    /* 航向每拍都向指定目标修正，归一化后按最短旋转方向计算 */
    motionState_.yawError = MathUtils::WrapPeriodicError(motionState_.target.yaw - currentPose.yaw, 2.0f * PI);

    /* 到达判定使用距最终目标的距离，与上方本拍参考位置误差区分 */
    motionState_.positionError = hypotf(motionState_.target.x - currentPose.x, motionState_.target.y - currentPose.y);

    /* 三路 P 反馈分别生成沿路径、横向和航向修正速度 */
    float alongCorrection = param_.alongKp * motionState_.alongError;
    float crossCorrection = param_.crossKp * motionState_.crossError;
    float yawVelocity = param_.yawKp * motionState_.yawError;

    /* 有效输入仍可能在乘法、加法中溢出，异常结果不能继续用于控制。 */
    if (!isfinite(motionState_.positionError) || !isfinite(alongCorrection) ||
        !isfinite(crossCorrection) || !isfinite(yawVelocity) ||
        !isfinite(hypotf(alongCorrection, crossCorrection)))
    {
        SetFault();
        return;
    }

    /* 按反馈向量的合成大小限幅，保持沿路径与横向修正的比例。 */
    LimitVector(alongCorrection, crossCorrection, param_.maxCorrectionVelocity);
    motionState_.alongCorrection = alongCorrection;
    motionState_.crossCorrection = crossCorrection;

    /* 梯形前馈只沿路径输出；与反馈合成后得到世界速度 v=u*(vRef+vAlong)+n*vCross。
     * 进入 Settling 时规划速度已为零，由位置反馈继续收敛到终点。 */
    const float alongVelocity = motionState_.referenceSpeed + alongCorrection;
    float worldVx = directionX_ * alongVelocity - directionY_ * crossCorrection;
    float worldVy = directionY_ * alongVelocity + directionX_ * crossCorrection;

    if (motionState_.phase == AutoMovePhase::Settling)
    {
        /* 位置和航向分别进入容差带后，将对应期望速度置零。
         * 实际输出仍经过下方的变化率限制，停稳判定由 UpdateArrival() 完成。 */
        if (motionState_.positionError <= param_.positionTolerance)
        {
            worldVx = 0.0f;
            worldVy = 0.0f;
        }
        if (fabsf(motionState_.yawError) <= param_.yawTolerance)
        {
            yawVelocity = 0.0f;
        }
    }

    /* 世界速度合成后的结果也须有效，再交给输出限制和坐标转换。 */
    if (!isfinite(worldVx) || !isfinite(worldVy) || !isfinite(hypotf(worldVx, worldVy)))
    {
        SetFault();
        return;
    }

    /* 限制最终速度及相邻两拍指令的变化，再用当前航向转成车体速度。 */
    UpdateOutput(worldVx, worldVy, yawVelocity, currentPose.yaw, dt);

    /* 位置、航向、实测速度和零输出须连续满足条件，才判定运动完成。 */
    UpdateArrival(measuredVelocity, dt);
}

void AutoMove::SetFault()
{
    motionState_.phase = AutoMovePhase::Fault;
    motionState_.worldVx = 0.0f;
    motionState_.worldVy = 0.0f;
    motionState_.velocity = ChassisBodyVelocity{};
    motionState_.settledTime = 0.0f;
}

void AutoMove::UpdateReference()
{
    /* 读取梯形规划器当前的参考状态，将距离、速度、时间和阶段
     * 分别复制到 AutoMove 的状态记录中，方便调试观察；这些不是实测值。 */
    const TrapezoidalProfileState &reference = distancePlanner_.State();
    motionState_.referenceDistance = reference.position;
    motionState_.referenceSpeed = reference.velocity;
    motionState_.planningTime = reference.elapsedTime;
    motionState_.planningPhase = reference.phase;

    if (motionState_.phase == AutoMovePhase::Settling)
    {
        motionState_.referencePose.x = motionState_.target.x;
        motionState_.referencePose.y = motionState_.target.y;
    }
    else
    {
        motionState_.referencePose.x = motionState_.startPose.x + directionX_ * motionState_.referenceDistance;
        motionState_.referencePose.y = motionState_.startPose.y + directionY_ * motionState_.referenceDistance;
    }
    motionState_.referencePose.yaw = motionState_.target.yaw;
}

void AutoMove::UpdateOutput(float worldVx, float worldVy, float yawVelocity,
                            float currentYaw, float dt)
{
    LimitVector(worldVx, worldVy, param_.maxOutputVelocity);
    yawVelocity = MathUtils::ClampAbs(yawVelocity, param_.maxYawVelocity);

    /* 在世界坐标系限制平移速度向量的变化，转动时仍保持同一条路径。 */
    const float currentSpeed = hypotf(motionState_.worldVx, motionState_.worldVy);
    const float targetSpeed = hypotf(worldVx, worldVy);
    const float rate = targetSpeed < currentSpeed ? param_.deceleration : param_.acceleration;
    float deltaVx = worldVx - motionState_.worldVx;
    float deltaVy = worldVy - motionState_.worldVy;
    const float deltaSpeed = hypotf(deltaVx, deltaVy);
    
    if (deltaSpeed > rate * dt)
    {
        LimitVector(deltaVx, deltaVy, rate * dt);
        worldVx = motionState_.worldVx + deltaVx;
        worldVy = motionState_.worldVy + deltaVy;
    }

    const float deltaYawVelocity = MathUtils::ClampAbs(yawVelocity - motionState_.velocity.wz, param_.yawAcceleration * dt);
    motionState_.velocity.wz += deltaYawVelocity;
    motionState_.worldVx = worldVx;
    motionState_.worldVy = worldVy;

    const float cosYaw = cosf(currentYaw);
    const float sinYaw = sinf(currentYaw);
    motionState_.velocity.vx = cosYaw * worldVx + sinYaw * worldVy;
    motionState_.velocity.vy = -sinYaw * worldVx + cosYaw * worldVy;
}

void AutoMove::UpdateArrival(const ChassisBodyVelocity &measuredVelocity, float dt)
{
    if (motionState_.phase != AutoMovePhase::Settling)
    {
        motionState_.settledTime = 0.0f;
        return;
    }

    const float measuredSpeed = hypotf(measuredVelocity.vx, measuredVelocity.vy);
    const bool arrived = motionState_.positionError <= param_.positionTolerance &&
                         fabsf(motionState_.yawError) <= param_.yawTolerance &&
                         measuredSpeed <= param_.speedTolerance &&
                         fabsf(measuredVelocity.wz) <= param_.yawSpeedTolerance &&
                         motionState_.worldVx == 0.0f && motionState_.worldVy == 0.0f &&
                         motionState_.velocity.wz == 0.0f;
    if (!arrived)
    {
        motionState_.settledTime = 0.0f;
        return;
    }

    motionState_.settledTime = fminf(param_.settleDuration, motionState_.settledTime + dt);
    if (motionState_.settledTime >= param_.settleDuration)
    {
        motionState_.phase = AutoMovePhase::Finished;
    }
}
