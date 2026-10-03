#ifndef CROSS_H
#define CROSS_H

#include <stdint.h>

#define CROSS_FRAME_COUNT  38U         // 38 帧，0~37
#define CROSS_JOINT_COUNT  12U         // 12 个关节，0~11

/*
 * 本地动作轨迹：38 帧 × 12 关节，单位 rad，模型坐标系（= Nano 下发格式）。
 *
 * 关节顺序（和 Nano 命令一致，和 Action_Goto() 的参数顺序相反！）：
 *   0..5  = 左腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 *   6..11 = 右腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 * 也就是每帧前 6 个送 hfdcan2（左腿），后 6 个送 hfdcan1（右腿）。
 *
 * 角度未取反：符号翻转、前倾、roll 偏置全部由
 * JetsonRobotBridge_ApplyModelTargets() 统一处理，所以这里保持和上位机
 * 完全一样的原始值，不要预先取反。
 *
 * 注意：这段数据原始 JSON 里的 joint_names 写的是 r_leg_* 在前，和上面
 * 的顺序相反。以代码里的顺序为准（前 6 个 = 左腿 / fdcan2）。若上机发现
 * 左右腿反了，把每一帧的前后 6 个数对调即可，不用改其它任何地方。
 */
extern const float cross_frames[CROSS_FRAME_COUNT][CROSS_JOINT_COUNT];

/*
 * 每个关节的角度倍率（12 个，顺序和 cross_frames 一致）。
 * 插值出来的模型角度先乘这个再交给电机，全 1.0 = 不改。
 * 可以在 ST-Link Live Expressions 里逐个关节调。
 *
 * 注意是"绝对角度 × 倍数"（绕模型零位缩放），不是"摆幅 × 倍数"：
 * 把某个关节设成 0 是把它拉回模型零位，不是"这个关节不动"。
 * 想改摆幅、同时保持它在站立姿态附近，得用"站立 + (轨迹-站立)×倍数"，
 * 那是另一套算法，需要的话再加。
 */

/* 按键调的时间倍数，1~5 整数（按键每按一次 +1，到 5 后绕回 1）。 */
extern uint8_t time_sca;

extern volatile float g_cross_scale[CROSS_JOINT_COUNT];

/*
 * 阻塞式回放整条轨迹。
 *
 *   time_scale : 时间倍数，整数，>= 1。1 = 原始速度（20ms 一帧，一条 760ms）；
 *                2 = 慢一倍（1520ms）；5 = 慢五倍（3800ms）。
 *   loops      : 重复遍数，必须 >= 1。
 *
 * 输出节拍固定 20ms 不变（和 Nano 下发指令的速率一致），倍数改变的是
 * 每拍推进多少帧，帧与帧之间做线性插值 —— 所以放慢时是平滑过渡，
 * 不是把某一帧多保持几十毫秒那种一卡一卡的跳变。
 *
 * 调用期间会占住调用者（主循环），约 760ms × time_scale × loops。
 * 只请在 Nano 没有控制时调用（g_debug_jetson_control_active == 0），
 * 否则两边会同时抢电机；而且回放结束后主循环一恢复，Jetson 看门狗可能
 * 因为指令过期而触发软复位。
 *
 * 例：Cross_Play(1U, 1U);   // 原始速度走一遍
 */
void Cross_Play(uint8_t time_scale, uint32_t loops);

#endif
