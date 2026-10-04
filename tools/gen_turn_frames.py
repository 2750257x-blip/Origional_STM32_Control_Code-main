#!/usr/bin/env python3
"""从 joint_frames_v1 JSON 生成 Core/Src/turn.c。

用法:
    python tools/gen_turn_frames.py "<input.json>"

JSON 格式 (joint_frames_v1):
    {"hz": 50.0, "joint_names": [...12...], "frames": [[...12...], ...]}

注意: 生成的数组按 JSON 原样落盘 —— 前 6 个数当作左腿 (模型下标 0~5),
后 6 个数当作右腿 (6~11)。JSON 里的 joint_names 写的是 r_leg_* 在前,
但那是导出脚本的命名, 和模型坐标系的左右约定相反, 不要按名字重排。
详见 turn.h 的说明。
"""

import json
import sys
from pathlib import Path

HEADER = """#include "turn.h"

#include "cross.h"

/* 本文件由 tools/gen_turn_frames.py 从 joint_angles_velocity_020_040_15s.json 生成，
 * 不要手改数据；要换轨迹就重新生成。
 *
 * {frame_count} 帧 × 12 关节，模型坐标系（= Nano 下发格式），单位 rad。
 * 采样率 {hz:g} Hz，每帧 20 ms，所以 time_scale = 1 时整条轨迹正好 {duration:g} 秒。
 *
 * 关节顺序（和 Cross_Play 的 cross_frames 一致，前 6 个左腿 / hfdcan2）：
 *   0..5  = 左腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 *   6..11 = 右腿 hip_pitch, hip_roll, hip_yaw, knee_pitch, ankle_pitch, ankle_roll
 * 角度未取反：符号翻转、前倾、roll 偏置全部由
 * JetsonRobotBridge_ApplyModelTargets() 统一处理。
 * 原始 JSON 的 joint_names 写的是 r_leg_* 在前，和这里相反；已按"前 6 个 = 左腿"
 * 落盘（和 cross_frames 同一套约定）。若上机发现左右腿反了，把每帧前后 6 个数
 * 对调即可，不用改别的地方。 */

/* 每个关节的角度倍率（12 个，顺序同上）。插值出来的模型角度先乘这个再交给电机，
 * 全 1.0 = 不改。可在 ST-Link Live Expressions 里逐个关节调。 */
volatile float g_turn_scale[TURN_JOINT_COUNT] = {{
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f
}};

const float turn_frames[TURN_FRAME_COUNT][TURN_JOINT_COUNT] = {{
"""

FOOTER = """}};

/* 固定转弯策略：按固定节拍回放上面这条转弯轨迹。
 * 实现完全复用 Cross_Play 的播放器（Trajectory_Play），只是数据源换成本文件的
 * turn_frames / g_turn_scale —— 所以插值方式、输出路径（ApplyModelTargets）、
 * 阻塞行为和 Cross_Play 一模一样。
 *
 *   time_scale : 时间倍数，整数 >= 1。1 = 原始速度（20 ms 一帧，一条 {duration:g} s）；
 *                2 = 慢一倍；越大越慢。放慢时帧间线性插值，是平滑的。
 *   loops      : 重复遍数，>= 1。
 *
 * 阻塞式：调用期间占住主循环约 {duration:g}s × time_scale × loops。
 * 只请在 Nano 没有控制时调用（g_debug_jetson_control_active == 0），否则两边
 * 会同时抢电机；而且回放结束后主循环一恢复，Jetson 看门狗可能因指令过期而软复位。
 *
 * 例：Turn_Play(1U, 1U);   // 原始速度转一圈
 */
void Turn_Play(uint8_t time_scale, uint32_t loops)
{{
    Trajectory_Play(turn_frames, TURN_FRAME_COUNT, g_turn_scale, time_scale, loops);
}}
"""


def c_float(value: float) -> str:
    """格式化成 C 的 float 字面量。%.9g 对 float32 是 round-trip 精确的；
    但整数值会输出成 "0"、"1"，加 f 后缀会变成非法整数常量，所以要补小数点。"""
    text = f"{value:.9g}"
    if ("." not in text) and ("e" not in text) and ("E" not in text):
        text += ".0"
    return text + "f"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    src = Path(sys.argv[1])
    data = json.loads(src.read_text(encoding="utf-8"))
    frames = data["frames"]
    hz = float(data.get("hz", 50.0))
    joint_names = data.get("joint_names")
    if joint_names is not None and len(joint_names) != 12:
        print(f"警告: joint_names 有 {len(joint_names)} 项, 期望 12", file=sys.stderr)

    width = len(frames[0])
    if width != 12:
        print(f"错误: 每帧 {width} 个关节, 期望 12", file=sys.stderr)
        return 1

    duration = len(frames) / hz
    out = [
        HEADER.format(frame_count=len(frames), hz=hz, duration=duration),
    ]
    for i, row in enumerate(frames):
        cells = ", ".join(c_float(float(v)) for v in row)
        out.append(f"    {{ {cells} }},   /* {i:>3} */\n")
    out.append(FOOTER.format(duration=duration))

    dest = Path("Core/Src/turn.c")
    dest.write_text("".join(out), encoding="utf-8")
    print(f"写入 {dest} ({len(frames)} 帧, {duration:g} s @ {hz:g} Hz)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
