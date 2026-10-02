#ifndef JETSON_ROBOT_BRIDGE_H
#define JETSON_ROBOT_BRIDGE_H

#include <stdint.h>

/*
 * Protocol joint order (model coordinates):
 *   0..5  = left  hip pitch, hip roll, hip yaw, knee pitch,
 *                   ankle pitch, ankle roll
 *   6..11 = right hip pitch, hip roll, hip yaw, knee pitch,
 *                   ankle pitch, ankle roll
 */
void JetsonRobotBridge_Init(void);
void JetsonRobotBridge_ProcessCommand(void);
uint8_t JetsonRobotBridge_SendState(void);
void JetsonRobotBridge_OnUsbReceive(uint8_t *data, uint32_t length);

/*
 * 停车读卡时的机身重摆（Nano 发动作号 7 / 8）。
 *
 * Latch 记下那一刻的姿态，Release 之后恢复真实上报；两者之间上报的都是
 * 锁存值。上位机的图卡几何按安装角 45° 标定，重摆把机身扳回那个姿态，
 * 所以锁存值在重摆之后恰好是真值 —— Nano 看到的姿态从头到尾没变。
 *
 * SetOffset 设踝关节偏置（弧度，0 = 撤销）。Nano 每 20ms 发来的关节目标
 * 被叠加上这个偏置，不是被接管 —— 所以 Nano 那边不用停发，也不会打架。
 *
 * ⚠️ SetOffset 必须是**逐步**调用的，别一次给满：0.2 rad 是 11°，踝关节
 * 一步跳过去会直接把人掀了。状态机里按小步逼近，具体见 main.c 的
 * ROBOT_CARD_TILT()。
 */
void JetsonRobotBridge_CardTiltLatch(void);
void JetsonRobotBridge_CardTiltRelease(void);
void JetsonRobotBridge_CardTiltSetOffset(float radians);

/* 重摆倾角总量，弧度，加在左右踝关节（ankle pitch）的**电机**目标上。
 * 单位是电机坐标系（已经过了 motor_direction_target 的反向），左右两条腿
 * 用同一个数 —— 镜像在电机那一侧，不在这里。
 *
 * 为什么是踝：脚掌是唯一接地的，踝关节一转整条机身就跟着转。
 *
 * 倾角不需要准：上位机那边的姿态是锁存的，重摆偏多少只影响识别好不好认，
 * 不影响几何。但**正负方向要试**：改成 +0.10 / -0.10 各跑一眼，看机身往
 * 哪边倒，然后把实测值和方向写在这行注释里。 */
#define CARD_TILT_ANKLE_RAD        0.20f
/* 每步弧度 / 每步间隔 ms。0.20 / 0.04 = 5 步、约 0.3 s。
 * 必须逐步给：0.2 rad 是 11°，踝关节一步跳过去会直接把人掀了。 */
#define CARD_TILT_STEP_RAD         0.04f
#define CARD_TILT_STEP_MS          60U

/* Values after URDF-to-motor direction conversion, visible over ST-Link. */
extern volatile float g_debug_motor_target[12];
extern volatile uint8_t g_debug_jetson_control_active;
extern volatile uint32_t g_debug_watchdog_trip_count;
extern volatile uint32_t g_debug_invalid_command_count;

#endif
