#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "vesc_motor.h"
#include "dji_6020.h"



extern "C" void StartChassisTask(void *argument)
{
    (void)argument;

    /* ---- VESC (FDCAN1)：扩展帧，每个电机一帧独立下发 ---- */
    VescMotors[0].init(&hfdcan1, 90);
    VescMotors[1].init(&hfdcan1, 69);
    VescMotors[2].init(&hfdcan1, 97);

    /* ---- DJI 6020 (FDCAN3)：标准帧，三个电机共用 0x1FF 一帧 ----
     * 数组下标按机械位置排，与 CAN ID 顺序无关：
     *   [0] 前轮 → 1 号（反馈 0x205）
     *   [1] 左轮 → 4 号（反馈 0x208）
     *   [2] 右轮 → 3 号（反馈 0x207）
     */
    Dji6020Motors[0].init(1);   // 前轮
    Dji6020Motors[1].init(4);   // 左轮
    Dji6020Motors[2].init(3);   // 右轮

    /* 一次性配好 FDCAN1 + FDCAN3 的滤波器并启动外设 */
    BSP_CAN::Init();

    /* VESC 下发分频计数（原因见下方注释） */
    uint32_t vescDivider = 0;

    for (;;)
    {
        /* ---- 6020：1 kHz 下发 ----
         * 6020 反馈本身就是 1 kHz，控制同频可以让"最新反馈 → 新指令"
         * 的延迟稳定在一个周期内，做闭环时相位滞后最小。
         */
        Dji6020Bus::Control();

        /* ---- VESC：降频到 100 Hz 下发 ----
         * FDCAN1 上三个电机是各自一帧，若也按 1 kHz 发就是 3000 帧/s；
         * 扩展帧约 131 bit、加上位填充约 150 bit，在 500 kbps 上要占用
         * 约 90% 总线负载 —— 会把总线打满并导致丢帧。
         * 降到 100 Hz 后约 9%，余量充足，且远小于 VESC 命令超时时间。
         */
        if (++vescDivider >= 10)
        {
            vescDivider = 0;

            VescMotors[0].setCurrent(100);   // 单电机测试值，保持现状
            VescMotors[1].setCurrent(0);
            VescMotors[2].setCurrent(0);
        }

        osDelay(1);   // configTICK_RATE_HZ = 1000 → 1 ms
    }
}
