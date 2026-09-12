/**
 *  【VESC CAN 协议说明】
 * ============================================================
 *  VESC 使用 CAN 扩展帧（29位ID），ID 布局：
 *
 *    Bits[7:0]  = VESC 节点 ID（0~255，每个电调唯一）
 *    Bits[15:8] = 命令类型（CAN_PACKET_ID 枚举值）
 *    Bits[28:16]= 固定为 0（VESC 协议未使用高位）
 *
 *  所以发送一条命令：
 *    Identifier = (node_id & 0xFF) | ((uint32_t)cmd << 8)
 *
 *  接收时解析：
 *    node_id = Identifier & 0xFF
 *    cmd     = (Identifier >> 8) & 0xFF
 *
 * ============================================================
 *  【极对数与 RPM 换算】
 *  VESC 内部使用"电气转速 eRPM"，与机械转速的关系：
 *    eRPM = 机械 RPM × 极对数(pairs_of_poles)
 *  本项目电机：24N28P → 极对数 = 28/2 = 14
 *  发送 RPM 指令时需乘以 14 转换为 eRPM 再发送。
 *  接收状态帧时，eRPM 除以 14 得到机械转速。
 *
 * ============================================================
 */

#pragma once  // 等价于 #ifndef 头文件保护，更简洁

#include "main.h"               // STM32 HAL 总头文件，包含 stm32h7xx_hal.h
#include "stm32h7xx_hal_fdcan.h"// FDCAN HAL 驱动（H7 专用，提供 FDCAN_HandleTypeDef）

/* ============================================================
 *  VESC CAN 命令类型枚举
 *  每个枚举值对应 CAN 扩展ID 的 Bits[15:8]，即"命令字"
 * ============================================================ */
enum class CanPacketID : uint8_t
{
    SET_DUTY                     = 0,   ///< 设置占空比（-1.0 ~ +1.0，协议内放大100000倍）
    SET_CURRENT                  = 1,   ///< 设置目标电流（接口 mA，协议原始值为 A×1000）
    SET_CURRENT_BRAKE            = 2,   ///< 设置制动电流（单位 mA，协议内放大1000倍）
    SET_RPM                      = 3,   ///< 设置目标转速（eRPM，= 机械RPM × 极对数）
    SET_POS                      = 4,   ///< 设置目标位置（度，协议内放大1000000倍）

    FILL_RX_BUFFER               = 5,   ///< 填充接收缓冲区（多帧传输用）
    FILL_RX_BUFFER_LONG          = 6,   ///< 填充长缓冲区（多帧传输用）
    PROCESS_RX_BUFFER            = 7,   ///< 处理接收缓冲区数据（多帧结束标志）
    PROCESS_SHORT_BUFFER         = 8,   ///< 处理短缓冲区数据

    STATUS                       = 9,   ///< 状态帧1：eRPM、电流、占空比
    SET_CURRENT_REL              = 10,  ///< 相对电流（-1.0~1.0，对应最大电流百分比）
    SET_CURRENT_BRAKE_REL        = 11,  ///< 相对制动电流
    SET_CURRENT_HANDBRAKE        = 12,  ///< 手刹电流（绝对值，mA）
    SET_CURRENT_HANDBRAKE_REL    = 13,  ///< 手刹相对电流（0~1.0）
    STATUS_2                     = 14,  ///< 状态帧2：安时消耗 / 安时回充
    STATUS_3                     = 15,  ///< 状态帧3：瓦时消耗 / 瓦时回充
    STATUS_4                     = 16,  ///< 状态帧4：MOSFET温度、电机温度、PID位置
    PING                         = 17,  ///< Ping（探测在线）
    PONG                         = 18,  ///< Pong（在线应答）
    DETECT_APPLY_ALL_FOC         = 19,  ///< FOC 自动检测并应用
    DETECT_APPLY_ALL_FOC_RES     = 20,  ///< FOC 自动检测结果
    CONF_CURRENT_LIMITS          = 21,  ///< 配置电流限制
    CONF_STORE_CURRENT_LIMITS    = 22,  ///< 存储电流限制到 EEPROM
    CONF_CURRENT_LIMITS_IN       = 23,  ///< 配置输入电流限制
    CONF_STORE_CURRENT_LIMITS_IN = 24,  ///< 存储输入电流限制
    CONF_FOC_ERPMS               = 25,  ///< 配置 FOC eRPM 参数
    CONF_STORE_FOC_ERPMS         = 26,  ///< 存储 FOC eRPM 参数
    STATUS_5                     = 27,  ///< 状态帧5：输入电压、里程计数
};

/* ============================================================
 *  电机实时接收数据结构体
 *  由 CAN_Rx_Handler 解析后填入，可直接读取
 * ============================================================ */
struct VescRxData
{
    // ------- 状态帧1 (STATUS) 解析字段 -------
    float eRpm;             ///< 电气转速 (eRPM)，= 机械RPM × 极对数
    float rpm;              ///< 机械转速 (RPM)，= eRPM / 极对数
    float duty;             ///< 当前占空比，范围 -1.0 ~ +1.0
    float totalCurrent;     ///< 总相电流 (A)

    // ------- 状态帧4 (STATUS_4) 解析字段 -------
    float pidPositionNow;   ///< 当前 PID 角度 (度)，范围 0~360，VESC 以50倍压缩存储
    float pidPositionLast;  ///< 上一次 PID 角度，用于检测跨零点方向

    // ------- 累计位置计算（基于 STATUS_4） -------
    int   turnCount;        ///< 圈数计数器（正转+1，反转-1）
    float totalPosition;    ///< 累计角度 (度) = turnCount×360 + pidPositionNow

    // ------- 其他（STATUS 中可选） -------
    float dutyCycle;        ///< 占空比副本（与 duty 相同，保持兼容）
    bool hasPosition;       ///< 是否已经收到过第一帧有效位置

    // 默认构造清零
    VescRxData()
        : eRpm(0.f), rpm(0.f), duty(0.f), totalCurrent(0.f)
        , pidPositionNow(0.f), pidPositionLast(0.f)
        , turnCount(0), totalPosition(0.f), dutyCycle(0.f), hasPosition(false)
    {}
};

/* ============================================================
 *  VescMotor 类：封装单个 VESC 电调的全部操作
 *
 *  【使用方法】
 *  ① 声明对象（可放全局或局部）：
 *       VescMotor motor1;
 *
 *  ② 初始化（绑定 CAN 外设句柄和节点 ID）：
 *       motor1.init(&hfdcan1, 10);   // 使用 FDCAN1，VESC 节点ID=10
 *
 *  ③ 在主循环中调用控制接口：
 *       motor1.setCurrent(5000);      // 5000 mA
 *       motor1.setRpm(300);           // 300 RPM（内部自动×极对数）
 *       motor1.setPwm(0.5);           // 50% 占空比
 *
 *  ④ 接收由 bsp_can 取帧后交给协议入口，不必在任务里自己调：
 *       VescMotor::ParseCanFeedback(identifier, data);
 *
 *  ⑤ 读取反馈数据：
 *       float speed = motor1.getRxData().rpm;
 *       float angle = motor1.getRxData().totalPosition;
 * ============================================================ */
class VescMotor
{
public:
    /* --------------------------------------------------------
     *  构造函数：不绑定外设，需显式调用 init()
     * -------------------------------------------------------- */
    VescMotor();

    /* --------------------------------------------------------
     *
     *  @param hfdcan   指向 HAL FDCAN 句柄（&hfdcan1 / &hfdcan2 / &hfdcan3）
     *  @param nodeId   VESC 节点 CAN ID（0~255），需与 VESC Tool 中配置的 ID 一致
     *
     *  FDCAN 句柄在调用本函数前必须已经由 CubeMX 生成的 MX_FDCANx_Init() 初始化。
     *  本驱动不负责滤波器配置，请在 MX_FDCANx_Init() 后额外调用：
     *    HAL_FDCAN_ConfigFilter / HAL_FDCAN_ConfigGlobalFilter
     *    HAL_FDCAN_Start
     *    HAL_FDCAN_ActivateNotification(..., FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
     * -------------------------------------------------------- */
    void init(FDCAN_HandleTypeDef* hfdcan, uint16_t nodeId);

    /**
     * @brief 设置电流（力矩控制模式）
     * @param current_mA  目标电流，单位 mA（毫安）
     *                    正值=正转驱动，负值=反转驱动
     *
     * 协议细节：
     *   VESC 协议 int32 = 电流(A) × 1000；接口传入 mA 后直接打包
     *   → data[0..3] = big-endian int32(current_mA)
     *   CAN ID: bits[7:0]=nodeId, bits[15:8]=CAN_PACKET_SET_CURRENT(1)
     */
    void setCurrent(int32_t current_mA);

    /**
     * @brief 设置转速（速度控制模式）
     * @param rpm  目标机械转速，单位 RPM
     *             正值=正转，负值=反转
     *
     * 协议细节：
     *   VESC 使用 eRPM，内部自动换算：eRPM = rpm × pairs_of_poles
     *   → data[0..3] = big-endian int32(eRPM)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_RPM(3)
     */
    void setRpm(int32_t rpm);

    /**
     * @brief 设置占空比（开环电压控制模式）
     * @param pwm  占空比，范围 -1.0（满反转）~ +1.0（满正转）
     *             0.0 = 停止（但不刹车）
     *
     * 协议细节：
     *   VESC 接收 int32 = pwm × 100000（放大10万倍存为整数）
     *   → data[0..3] = big-endian int32(pwm × 100000)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_DUTY(0)
     */
    void setPwm(double pwm);

    /**
     * @brief 设置目标位置（位置控制模式）
     * @param pos  目标位置（整数，单位由 VESC 内部配置决定，通常为角度×1e6）
     *
     * 协议细节：
     *   → data[0..3] = big-endian int32(pos)，直接发送，不做缩放
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_POS(4)
     */
    void setPos(int32_t pos);

    /**
     * @brief 设置制动电流（再生制动 / 电阻制动）
     * @param current_mA  制动电流，单位 mA，应为正值
     *
     * 协议细节：
     *   VESC 协议 int32 = 电流(A) × 1000；接口传入 mA 后直接打包
     *   → data[0..3] = big-endian int32(current_mA)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_BRAKE(2)
     *
     * 注意：与 setCurrent(负值) 的区别在于：
     *   setCurrent 是四象限电流控制（可加速也可制动）
     *   setBrakeCurrent 是专用制动指令（只制动，不驱动）
     */
    void setBrakeCurrent(int32_t current_mA);

    /**
     * @brief 设置手刹电流（绝对值制动，适合停车保持）
     * @param current_mA  手刹电流，单位 mA，正值
     *
     * 协议细节：
     *   int32 = 电流(A) × 1000；接口传入 mA 后直接打包
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_HANDBRAKE(12)
     *
     * 手刹 vs 制动电流：
     *   手刹以位置保持为目标，制动以减速为目标
     */
    void setHandbrakeCurrent(int32_t current_mA);

    /**
     * @brief 设置相对手刹电流（比例制动）
     * @param relative  相对电流比例，范围 0.0 ~ 1.0
     *                  0.0 = 无制动，1.0 = 最大制动电流
     *
     * 协议细节：
     *   int32 = relative × 100000
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_HANDBRAKE_REL(13)
     */
    void setHandbrakeCurrentRel(float relative);

    /**
     * @brief VESC 协议入口：消化一帧已取出的 CAN 反馈
     *
     * 由 bsp_can 在 FIFO1 取帧后调用。本函数不碰 HAL / FIFO，
     * 只根据扩展 ID 找到对应电机并解析 8 字节负载。
     *
     * @param identifier  29 位扩展 ID：低 8 位=节点号，第 8~15 位=状态类型
     * @param data        8 字节数据
     */
    static void ParseCanFeedback(uint32_t identifier, const uint8_t data[8]);

    /* ========================================================
     *  数据访问接口（读取反馈）
     * ======================================================== */

    /**
     * @brief 获取接收数据的 const 引用（只读）
     * @return VescRxData 结构体引用，包含转速、电流、位置等信息
     */
    const VescRxData& getRxData() const { return rxData_; }

    /**
     * @brief 获取节点 ID
     */
    uint16_t getNodeId() const { return nodeId_; }

    /**
     * @brief 获取绑定的 FDCAN 句柄指针
     */
    FDCAN_HandleTypeDef* getFdcan() const { return hfdcan_; }

private:
    /* --------------------------------------------------------
     *  私有成员变量
     * -------------------------------------------------------- */

    FDCAN_HandleTypeDef* hfdcan_;  ///< 绑定的 FDCAN 外设句柄
    uint16_t             nodeId_;  ///< 本电调的 CAN 节点 ID（0~255）
    VescRxData           rxData_;  ///< 最新接收并解析的状态数据

    /**
     * @brief 极对数（pole pairs）
     * 24N28P 电机：28极 / 2 = 14 极对数
     * 此值用于 eRPM ↔ 机械RPM 换算，建议以构造参数或常量传入
     * 当前按原代码硬编码为 14
     */
    static constexpr int POLE_PAIRS = 14;

    /* --------------------------------------------------------
     *  私有辅助函数：构建并发送 FDCAN 扩展数据帧
     *
     *  @param cmd    命令类型（CanPacketID 枚举）
     *  @param data   4字节数据负载（big-endian int32，其余字节填0）
     *
     *  FDCAN 帧参数说明：
     *    IdType      = FDCAN_EXTENDED_ID   → 29位扩展帧，VESC 协议要求
     *    TxFrameType = FDCAN_DATA_FRAME    → 数据帧（非远程帧）
     *    DataLength  = FDCAN_DLC_BYTES_8   → DLC=8，固定8字节负载
     *    Identifier  = nodeId_ | (cmd<<8)  → VESC 协议 ID 格式
     *    ErrorStateIndicator = FDCAN_ESI_ACTIVE  (默认，无错误)
     *    BitRateSwitch       = FDCAN_BRS_OFF     (经典CAN，不切换比特率)
     *    FDFormat            = FDCAN_CLASSIC_CAN (不启用 FD 模式)
     *    TxEventFifoControl  = FDCAN_NO_TX_EVENTS(不记录发送事件)
     *    MessageMarker       = 0
     * -------------------------------------------------------- */
    void sendFrame(CanPacketID cmd, const uint8_t data[8]);

    /**
     * @brief 将 int32_t 按大端序拆分到 data[0..3]
     * @param val   要拆分的32位整数（补码）
     * @param data  输出字节数组（至少4字节），data[0]=高字节
     *
     * VESC 协议统一使用大端序（Big-Endian / MSB first）：
     *   data[0] = (val >> 24) & 0xFF  // 最高字节
     *   data[1] = (val >> 16) & 0xFF
     *   data[2] = (val >>  8) & 0xFF
     *   data[3] = (val      ) & 0xFF  // 最低字节
     */
    static void packInt32BigEndian(int32_t val, uint8_t* data);

    /**
     * @brief 节点号已匹配后，按状态类型解析 8 字节负载到 rxData_
     */
    void parseStatusPayload(CanPacketID cmd, const uint8_t data[8]);
};

/* ============================================================
 *  全局电机数组（与原代码 Vesc_Motor_U8[4] 对应）
 *  下标 0~2 对应三个电机，使用前需各自调用 init()
 * ============================================================ */
extern VescMotor VescMotors[3];
