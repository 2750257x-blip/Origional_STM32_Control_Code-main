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

/* 前倾 + 反馈冻结 的控制接口（由 Handle_Action_Request 调用） */
void JetsonRobotBridge_SetLean(uint8_t active);
void JetsonRobotBridge_CaptureFrozenState(void);

/* 前倾斜坡走完了没有。
 *
 * LEAN/RESTORE 故意不占 robot_state（要跨着后面的手臂动作持续存在），于是
 * ROBOT_Comms_Service 里"回到 IDLE 就算做完"那条规则一收下就成立 —— DONE
 * 会在几毫秒后报到上位机，而前倾要 580ms 才爬到位，上位机就会在机身还在
 * 倒的过程中开始读卡。DONE 因此要等这个。
 *
 * 没在做前倾/恢复时 lean_applied 和 lean_target 都是 0，恒为真，
 * 所以对 1/2/5/6 那条路径没有影响。 */
uint8_t JetsonRobotBridge_LeanSettled(void);

/* Values after URDF-to-motor direction conversion, visible over ST-Link. */
extern volatile float g_debug_motor_target[12];
extern volatile uint8_t g_debug_jetson_control_active;
extern volatile uint32_t g_debug_watchdog_trip_count;
extern volatile uint32_t g_debug_invalid_command_count;
extern volatile float   g_debug_pose_lean_applied;
extern volatile uint8_t g_debug_feedback_freeze;

#endif
