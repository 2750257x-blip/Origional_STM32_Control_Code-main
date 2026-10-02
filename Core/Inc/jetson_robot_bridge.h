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
 * 模型停下后进入站姿，而那个站姿在实机上后仰约 20° —— 相机因此只朝下 25°，
 * 图卡看不清。重摆把机身扳回安装姿态 45°。
 *
 * 「冻结」= 保留**加偏置之前的最后一个真值**。偏置落地那一刻「站着后仰」这个
 * 事实就不成立了，所以只能在它还成立的时候记下来，之后一直报这个值。模型不
 * 能知道自己被搬动了，否则它会去纠回来。
 *
 * ⚠️ 要冻的不只是四元数：Nano 的策略观测里还有加速度计、角速度和关节角，
 *    只要还有一条通道在说真话，偏置就藏不住（加速度计静止时读的就是重力，
 *    它和四元数说的是同一件事）。所以 Latch 把姿态、加速度、角速度一起冻住，
 *    两个腿 pitch 关节的关节角/角速度则从上报值里把偏置减掉 —— 减在电机
 *    坐标系、过 sign 之前，还原出来的正是模型自己下发的那个目标。
 *
 * ⚠️ 锁存点必须是**机身站定之后**那个站姿，不是走路中的某一瞬：模型停车后
 *    期待的是站姿，喂它一个步态相位它会以为自己在走路。所以先等站定
 *    （CardTiltBodySettled），再 Latch。状态机见 main.c 的 ROBOT_CARD_TILT()。
 *
 * ⚠️ 冻的是「加偏置之前」，而机身这时已经被搬到 45° 了 —— 上报值和相机真实
 *    姿态**不一致**，这是故意的。所以视觉的图卡几何不能用这个上报值，得用
 *    静态安装角；run_policy_vision.py 里按停车窗口区分。别把这条"修掉"。
 *
 * SetOffset 设腿 pitch 偏置（弧度，0 = 撤销）。Nano 每 20ms 发来的关节目标
 * 被叠加上这个偏置，不是被接管 —— 所以 Nano 那边不用停发，也不会打架。
 *
 * ⚠️ SetOffset 必须是**逐步**调用的，别一次给满：0.35 rad 是 20°，一步跳
 *    过去会直接把人掀了。状态机里按小步逼近，见 main.c 的 ROBOT_CARD_TILT()。
 */
void JetsonRobotBridge_CardTiltLatch(void);
void JetsonRobotBridge_CardTiltRelease(void);
void JetsonRobotBridge_CardTiltSetOffset(float radians);

/* 机身站定了没有：陀螺仪连续 CARD_TILT_QUIET_MS 都在门限以下。
 * 门限抄的是 imu_filter.py 启动检查里那个 0.35 rad/s —— 那是验证过的
 * 「这台机器人站着不动」的判据。等超过 CARD_TILT_SETTLE_MAX_MS 就按现状认，
 * 免得机身一直微晃把整个图卡窗口耗掉。 */
void JetsonRobotBridge_CardTiltSettleReset(void);
uint8_t JetsonRobotBridge_CardTiltBodySettled(void);
#define CARD_TILT_QUIET_RAD_S      0.35f
#define CARD_TILT_QUIET_MS         150U
#define CARD_TILT_SETTLE_MAX_MS    600U

/* 偏置的变化率多久算过期。状态机每 CARD_TILT_STEP_MS 调一次 SetOffset，
 * 两次没来就说明斜坡停了 —— 之后再减这个变化率就是纯偏差。 */
#define CARD_TILT_RATE_STALE_MS    (2U * CARD_TILT_STEP_MS)

/* 当前已经施加的偏置。状态机用它做起点，别让重摆从 0 重新爬 —— 万一上一次的
 * 恢复被丢掉，偏置还在身上，从 0 开始就是一步 20° 的跳变。 */
float JetsonRobotBridge_CardTiltGetOffset(void);

/* 重摆倾角总量，弧度，加在**左右 leg pitch** 两个关节的**电机**目标上。
 * 单位是电机坐标系（已经过了 motor_direction_target 的反向），左右两条腿
 * 用同一个数 —— 镜像在电机那一侧，不在这里。
 *
 * 为什么是 leg pitch：停车的后仰就是这两个关节造成的，把同一个偏置加回去
 * 是最直接的抵消。倾角**不需要准**——上位机那边的姿态是锁存的，重摆偏多少
 * 只影响识别好不好认，不影响几何；而且它就是个固定偏置，不跟踪任何东西。
 *
 * ⚠️ **正负方向要台架试**：改成 +0.10 / -0.10 各跑一眼，看机身往哪边倒，
 *    然后把实测的方向和数值写在这行注释里。 */
#define CARD_TILT_LEG_PITCH_RAD    0.35f
/* 每步弧度 / 每步间隔 ms。0.35 / 0.05 = 7 步、约 0.4 s。
 * 必须逐步给：一步跳 20° 会直接把人掀了。 */
#define CARD_TILT_STEP_RAD         0.05f
#define CARD_TILT_STEP_MS          60U

/* Values after URDF-to-motor direction conversion, visible over ST-Link. */
extern volatile float g_debug_motor_target[12];
extern volatile uint8_t g_debug_jetson_control_active;
extern volatile uint32_t g_debug_watchdog_trip_count;
extern volatile uint32_t g_debug_invalid_command_count;

#endif
