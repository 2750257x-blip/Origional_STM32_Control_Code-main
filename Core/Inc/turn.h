#ifndef TURN_H
#define TURN_H

#include <stdint.h>

#define TURN_FRAME_COUNT  751U         // 751 帧，0~750
#define TURN_JOINT_COUNT  12U          // 12 个关节，0~11

/*
 * 固定转弯轨迹：751 帧 × 12 关节，单位 rad，模型坐标系（= Nano 下发格式）。
 * 采样率 50 Hz，每帧 20 ms，整条轨迹 15.02 s。
 * 数据由 tools/gen_turn_frames.py 从 joint_angles_velocity_020_040_15s.json 生成。
 *
 * 关节顺序（和 cross.h 的 cross_frames 完全一致，不要搞混）：
 *   0..5  = 左腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 *   6..11 = 右腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 * 也就是每帧前 6 个送 hfdcan2（左腿），后 6 个送 hfdcan1（右腿）。
 *
 * 角度未取反：符号翻转、前倾、roll 偏置全部由
 * JetsonRobotBridge_ApplyModelTargets() 统一处理，所以这里保持和上位机一样的
 * 原始值，不要预先取反。
 *
 * 注意：原始 JSON 里的 joint_names 写的是 r_leg_* 在前，和上面的顺序相反。
 * 已按"前 6 个 = 左腿"落盘（和 cross_frames 同一套约定，cross.h 里有同样的说明）。
 * 若上机发现左右腿反了，把每一帧的前后 6 个数对调即可，不用改其它任何地方。
 */
extern const float turn_frames[TURN_FRAME_COUNT][TURN_JOINT_COUNT];

/* 每个关节的角度倍率（12 个，顺序同 turn_frames）。全 1.0 = 不改。
 * 可以在 ST-Link Live Expressions 里逐个关节调。 */
extern volatile float g_turn_scale[TURN_JOINT_COUNT];

/*
 * 固定转弯策略：回放整条转弯轨迹。
 *
 *   time_scale : 时间倍数，整数 >= 1。1 = 原始速度（20ms 一帧，一条 15.02s）；
 *                2 = 慢一倍；越大越慢。放慢时帧间线性插值，是平滑过渡。
 *   loops      : 重复遍数，必须 >= 1。
 *
 * 内部复用 Cross_Play 同一个播放器（Trajectory_Play），输出路径和 Cross_Play
 * 完全一致，只是数据源换成 turn_frames。
 *
 * 调用期间会占住调用者（主循环），约 15.02s × time_scale × loops。
 * 只请在 Nano 没有控制时调用（g_debug_jetson_control_active == 0），否则两边
 * 会同时抢电机；而且回放结束后主循环一恢复，Jetson 看门狗可能因为指令过期
 * 而触发软复位。
 *
 * 例：Turn_Play(1U, 1U);   // 原始速度转一遍
 */
void Turn_Play(uint8_t time_scale, uint32_t loops);

#endif
