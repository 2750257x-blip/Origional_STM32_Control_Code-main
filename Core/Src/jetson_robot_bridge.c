#include "jetson_robot_bridge.h"

#include "fdcan.h"
#include "imu.h"
#include "jetson_protocol.h"
#include "jetson_usb_cdc.h"
#include "motor_el05.h"
#include "protocol.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define JETSON_COMMAND_WATCHDOG_MS 100U
#define IMU_FRESHNESS_MS           50U
#define MAX_GAIN_SCALE             2.0f

static uint32_t last_applied_command_count;

/* 当前生效的腿 pitch 偏置，由状态机逐步逼近 CARD_TILT_LEG_PITCH_RAD */
static volatile float card_tilt_offset_rad;
/* 偏置的当前变化率 (rad/s)：上报关节角速度时要把这一份减掉 */
static float card_tilt_rate_rad_s;
static uint32_t card_tilt_rate_tick_ms;
/* 1 = 上报锁存的旧状态而不是真实状态 */
static volatile uint8_t card_tilt_latched;
/* 站定那一刻锁存的姿态/加速度/角速度，重摆期间原样上报 */
static float card_tilt_quat[4];
static float card_tilt_accel[3];
static float card_tilt_gyro[3];

/* ---- 反馈前向预测滤波 ---- */
#define SMOOTHING_ALPHA         0.0f     /* 0~1: 越大越跟随预测值 */
#define FRAME_ANGLE_LIMIT       0.5f     /* 反馈偏离目标的最大单帧变化量 (rad) */
#define MAX_PREDICT_DT_MS       50U      /* 预测dt上限 (ms)，防止卡顿瞬间跳变 */

/* Nano 下发的目标位置，用作平滑参考（无滞后、无噪声） */
static float target_position[PROTOCOL_NUM_JOINTS];
static uint32_t last_send_tick_ms;

volatile float g_debug_motor_target[PROTOCOL_NUM_JOINTS];
volatile uint8_t g_debug_jetson_control_active;
volatile uint32_t g_debug_watchdog_trip_count;
volatile uint32_t g_debug_invalid_command_count;

extern volatile uint32_t system_control_cycle;
 
const float kp_add = 1.5f;
const float kd_add = 2.1f;
static float limit_gain_scale(float scale)
{
    if (!isfinite(scale) || (scale < 0.0f)) {
        return 0.0f;
    }
    if (scale > MAX_GAIN_SCALE) {
        return MAX_GAIN_SCALE;
    }
    return scale;
}

/* 协议下标 → 电机 ID：0..5 左腿，6..11 右腿。
 *
 * 下发和上报共用这一份表。偏置是按电机 ID 加的，上报要按同一个 ID 减掉 ——
 * 两份表一旦漂开，减掉的就不是加上的那个，而且是静默的。 */
static const uint8_t k_left_motor_id[6] = {
    l_leg_pitch, l_leg_roll, l_leg_yaw,
    l_knee_pitch, l_ankle_pitch, l_ankle_roll
};
static const uint8_t k_right_motor_id[6] = {
    r_leg_pitch, r_leg_roll, r_leg_yaw,
    r_knee_pitch, r_ankle_pitch, r_ankle_roll
};

static uint8_t protocol_motor_id(uint8_t index)
{
    return (index < 6U) ? k_left_motor_id[index] : k_right_motor_id[index - 6U];
}

static float motor_direction_target(uint8_t joint_index, float model_target)
{
    /* These four joint directions are reversed between the URDF and motors. */
    if ((joint_index == 0U) || (joint_index == 4U) ||
        (joint_index == 6U) || (joint_index == 10U)) {
        return -model_target;
    }
    return model_target;
}

static bool command_targets_are_valid(const RobotCommandPayload *command)
{
    uint8_t index;

    for (index = 0U; index < PROTOCOL_NUM_JOINTS; ++index) {
        if (!isfinite(command->joint_target[index])) {
            return false;
        }
    }
    return true;
}

/* 重摆期间给腿 pitch 关节的偏置。
 *
 * 是腿 pitch，不是踝：停车的后仰来自 leg pitch 那两个关节，所以把同一个偏置
 * 加回去就是最直接的抵消。
 *
 * 按电机 ID 判，不按协议下标：本文件开头那段注释说协议 0..5 是左腿，而上位机
 * 的 config.JOINT_NAMES 是右腿在前，两边对不上，按电机 ID 判就不用先弄清谁对。
 */
static float card_tilt_offset(uint8_t motor_id)
{
    if ((motor_id != l_leg_pitch) && (motor_id != r_leg_pitch)) {
        return 0.0f;
    }
    return card_tilt_offset_rad;
}

/* 同一个偏置的变化率。位置减偏置、速度减变化率，两者才自洽 ——
 * 0.35 rad / 0.4 s 是 0.83 rad/s，比站着不动时关节的真实速度大得多。
 *
 * ⚠️ 只有刚调过偏置才算还在动。状态机走完就没入再调 SetOffset 了，这个变化率
 *    必须自己过期 —— 留在那儿等于给这两个关节的上报速度加一个**永久**的
 *    0.83 rad/s 偏差，而上位机正是靠 qd 判断"停稳了没有"（阈值 0.2）：偏着
 *    0.83，下一张卡会在 wait_stop 里一直等不到 stopped，最后超时把进程打断。 */
static float card_tilt_offset_rate(uint8_t motor_id)
{
    if ((motor_id != l_leg_pitch) && (motor_id != r_leg_pitch)) {
        return 0.0f;
    }
    if ((HAL_GetTick() - card_tilt_rate_tick_ms) > CARD_TILT_RATE_STALE_MS) {
        return 0.0f;
    }
    return card_tilt_rate_rad_s;
}

static void apply_position_targets(const RobotCommandPayload *command)
{
    static const float base_kp[6] = {
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f
    };
    static const float base_kd[6] = {
        2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f
    };
    // float kp_scale = limit_gain_scale(command->kp_scale);
    // float kd_scale = limit_gain_scale(command->kd_scale);
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    uint8_t index;

    for (index = 0U; index < 6U; ++index) {
        float target = motor_direction_target(index, command->joint_target[index]);
        //  if(index == 0U) {
        //     target = target - 0.08f;
        // }
        target += card_tilt_offset(k_left_motor_id[index]);
        g_debug_motor_target[index] = target;
        Motor_limitCtrl_float(
            &hfdcan2,
            k_left_motor_id[index],
            target,
            0.0f,         
            base_kp[index] * kp_add * kp_scale,
            base_kd[index] * kd_add * kd_scale);
    }

    for (index = 0U; index < 6U; ++index) {
        uint8_t protocol_index = (uint8_t)(index + 6U);
        float target = motor_direction_target(
            protocol_index,
            command->joint_target[protocol_index]);
        // if(index == 0U) {
        //     target = target +  0.08f;
        // }
        target += card_tilt_offset(k_right_motor_id[index]);
        g_debug_motor_target[protocol_index] = target;
        Motor_limitCtrl_float(
            &hfdcan1,
            k_right_motor_id[index],
            target,
            0.0f,
            base_kp[index] * kp_add * kp_scale,
            base_kd[index] * kd_add * kd_scale);
    }
    system_control_cycle ++;
}

static void stop_all_motors(void)
{
    uint8_t index;

    for (index = 0U; index < 6U; ++index) {
        EL05_Motor_Stop(&hfdcan1, (uint8_t)(r_leg_pitch + index));
        EL05_Motor_Stop(&hfdcan2, (uint8_t)(l_leg_pitch + index));
    }
}

void JetsonRobotBridge_Init(void)
{
    Protocol_Init();
    last_applied_command_count = 0U;
    memset((void *)g_debug_motor_target, 0, sizeof(g_debug_motor_target));
    g_debug_jetson_control_active = 0U;
    g_debug_watchdog_trip_count = 0U;
    g_debug_invalid_command_count = 0U;
    memset(target_position, 0, sizeof(target_position));
    last_send_tick_ms = HAL_GetTick();
    card_tilt_offset_rad = 0.0f;
    card_tilt_rate_rad_s = 0.0f;
    card_tilt_rate_tick_ms = HAL_GetTick();
    card_tilt_latched = 0U;
    memset(card_tilt_quat, 0, sizeof(card_tilt_quat));
    memset(card_tilt_accel, 0, sizeof(card_tilt_accel));
    memset(card_tilt_gyro, 0, sizeof(card_tilt_gyro));
}

void JetsonRobotBridge_CardTiltLatch(void)
{
    __disable_irq();
    memcpy(card_tilt_quat, &MotorIMU_Packet_float[30], sizeof(card_tilt_quat));
    memcpy(card_tilt_accel, &MotorIMU_Packet_float[24], sizeof(card_tilt_accel));
    memcpy(card_tilt_gyro, &MotorIMU_Packet_float[27], sizeof(card_tilt_gyro));
    card_tilt_latched = 1U;
    __enable_irq();
}

void JetsonRobotBridge_CardTiltRelease(void)
{
    card_tilt_latched = 0U;
}

float JetsonRobotBridge_CardTiltGetOffset(void)
{
    return card_tilt_offset_rad;
}

void JetsonRobotBridge_CardTiltSetOffset(float radians)
{
    uint32_t now_ms;
    uint32_t dt_ms;

    if (!isfinite(radians)) {
        return;
    }
    if (radians > CARD_TILT_LEG_PITCH_RAD) {
        radians = CARD_TILT_LEG_PITCH_RAD;
    }
    if (radians < -CARD_TILT_LEG_PITCH_RAD) {
        radians = -CARD_TILT_LEG_PITCH_RAD;
    }
    now_ms = HAL_GetTick();
    dt_ms = now_ms - card_tilt_rate_tick_ms;
    if (dt_ms > 0U) {
        card_tilt_rate_rad_s =
            (radians - card_tilt_offset_rad) / ((float)dt_ms * 0.001f);
        card_tilt_rate_tick_ms = now_ms;
    }
    card_tilt_offset_rad = radians;
}

/* 一次停车读卡的等待从这里重新开始。每次动作 7 都要清，不然上一张卡留下的
 * 时间戳会让这一次在第一帧就判成"已经站定"。 */
static uint32_t card_tilt_settle_since_ms;
static uint32_t card_tilt_quiet_since_ms;

void JetsonRobotBridge_CardTiltSettleReset(void)
{
    uint32_t now_ms = HAL_GetTick();
    card_tilt_settle_since_ms = now_ms;
    card_tilt_quiet_since_ms = now_ms;
}

uint8_t JetsonRobotBridge_CardTiltBodySettled(void)
{
    uint32_t now_ms = HAL_GetTick();
    float gx;
    float gy;
    float gz;

    __disable_irq();
    gx = MotorIMU_Packet_float[27];
    gy = MotorIMU_Packet_float[28];
    gz = MotorIMU_Packet_float[29];
    __enable_irq();

    if (sqrtf(gx * gx + gy * gy + gz * gz) > CARD_TILT_QUIET_RAD_S) {
        card_tilt_quiet_since_ms = now_ms;
    }
    if ((now_ms - card_tilt_quiet_since_ms) >= CARD_TILT_QUIET_MS) {
        return 1U;
    }
    /* 一直微晃也不能把整个图卡窗口耗掉：到点就按现状锁。 */
    return (uint8_t)((now_ms - card_tilt_settle_since_ms)
                     >= CARD_TILT_SETTLE_MAX_MS);
}

void JetsonRobotBridge_ProcessCommand(void)
{
    RobotCommandPayload command;
    uint32_t now_ms = HAL_GetTick();
    uint32_t command_count_before;
    uint32_t command_count_after;
    bool fresh;

    /* Ensure the copied payload and its debug counter refer to the same frame. */
    do {
        command_count_before = g_debug_command_count;
        fresh = Protocol_GetFreshCommand(
            now_ms,
            JETSON_COMMAND_WATCHDOG_MS,
            &command);
        command_count_after = g_debug_command_count;
    } while (command_count_before != command_count_after);

    if (!fresh) {
        if (g_debug_jetson_control_active != 0U) {
            stop_all_motors();
            g_debug_jetson_control_active = 0U;
            ++g_debug_watchdog_trip_count;
        }
        return;
    }

    if (command_count_after == last_applied_command_count) {
        return;
    }
    last_applied_command_count = command_count_after;

    if (((command.command_flags & COMMAND_ENABLE) == 0U) ||
        ((command.command_flags & COMMAND_ESTOP) != 0U)) {
        if (g_debug_jetson_control_active != 0U) {
            stop_all_motors();
            g_debug_jetson_control_active = 0U;
        }
        return;
    }

    if (!command_targets_are_valid(&command)) {
        if (g_debug_jetson_control_active != 0U) {
            stop_all_motors();
            g_debug_jetson_control_active = 0U;
        }
        ++g_debug_invalid_command_count;
        return;
    }

    if (g_debug_jetson_control_active == 0U) {
        motor_enable();
        g_debug_jetson_control_active = 1U;
    }
    apply_position_targets(&command);
    memcpy(target_position, command.joint_target, sizeof(target_position));
}

uint8_t JetsonRobotBridge_SendState(void)
{
    RobotStatePayload state;
    float feedback[34];
    uint16_t ready_flags;
    uint16_t fault_flags;
    uint16_t mode_flags;
    uint8_t imu_flags;
    uint8_t index;

    memset(&state, 0, sizeof(state));

    __disable_irq();
    memcpy(feedback, MotorIMU_Packet_float, sizeof(feedback));
    ready_flags = motor_status_ready;
    fault_flags = motor_status_fault;
    mode_flags = motor_status_mode;
    imu_flags = imu_data_ready;
    __enable_irq();

    state.timestamp_us = HAL_GetTick() * 1000U;

    /* Δt 用于速度外推：距上次发送的时间差，上限保护防跳变 */
    uint32_t now_tick = HAL_GetTick();
    uint32_t delta_t_ms = now_tick - last_send_tick_ms;
    if (delta_t_ms > MAX_PREDICT_DT_MS) delta_t_ms = 0U;  // 首次或卡顿时不预测
    last_send_tick_ms = now_tick;
    float delta_t = (float)delta_t_ms * 0.001f;

    for (index = 0U; index < PROTOCOL_NUM_JOINTS; ++index) {
        float sign = ((index == 0U) || (index == 4U) ||
                      (index == 6U) || (index == 10U)) ? -1.0f : 1.0f;

        /* 原始反馈（已转模型坐标系）。偏置是我们内部加的，模型不该看见：
         * 在电机坐标系里、过 sign 之前减掉，还原出来的就是模型自己下发的
         * 那个目标。 */
        uint8_t motor_id = protocol_motor_id(index);
        float raw_pos = sign * (feedback[index * 2U]
                                - card_tilt_offset(motor_id));
        float raw_vel = sign * (feedback[index * 2U + 1U]
                                - card_tilt_offset_rate(motor_id));

        /* 1. 速度外推 —— 补偿1帧滞后：p_pred = p_raw + v * Δt */
        float predicted = raw_pos + raw_vel * delta_t;

        /* 2. 向目标位置平滑 —— 以外推为主，以目标为参考抑制噪声 */
        // float corrected = SMOOTHING_ALPHA * predicted
        //                 + (1.0f - SMOOTHING_ALPHA) * target_position[index];
        float corrected = SMOOTHING_ALPHA * predicted
                        + (1.0f - SMOOTHING_ALPHA) * raw_pos;


        /* 3. 单帧限幅 —— 反馈偏离目标不超过限制值，过滤跳变 */
        float delta = corrected - target_position[index];
        if (delta > FRAME_ANGLE_LIMIT)  delta = FRAME_ANGLE_LIMIT;
        if (delta < -FRAME_ANGLE_LIMIT) delta = -FRAME_ANGLE_LIMIT;
        corrected = target_position[index] + delta;

        state.joint_position[index] = corrected;
        state.joint_velocity[index] = raw_vel;
    }
    if (card_tilt_latched != 0U) {
        /* 重摆期间上报站定那一刻的姿态/加速度/角速度。模型不能知道自己被搬动
         * 了 —— 静止时加速度计读的就是重力，它和四元数说的是同一件事，只冻
         * 四元数等于给模型两个自相矛盾的姿态。
         *
         * ⚠️ 这个姿态是**加偏置之前**的，而机身这时已经扳到安装姿态了：上报
         *    值和相机真实朝向不一致，是故意的。视觉的图卡几何不能用它。 */
        memcpy(state.accel_m_s2, card_tilt_accel, sizeof(state.accel_m_s2));
        memcpy(state.gyro_rad_s, card_tilt_gyro, sizeof(state.gyro_rad_s));
        memcpy(state.orientation_wxyz, card_tilt_quat,
               sizeof(state.orientation_wxyz));
    } else {
        memcpy(state.accel_m_s2, &feedback[24], sizeof(state.accel_m_s2));
        memcpy(state.gyro_rad_s, &feedback[27], sizeof(state.gyro_rad_s));
        memcpy(state.orientation_wxyz, &feedback[30], sizeof(state.orientation_wxyz));
    }

    if (mode_flags == 0x0FFFU) {
        state.status_flags |= STATE_MOTORS_ENABLED;
    }
    if (fault_flags != 0U) {
        state.status_flags |= STATE_FAULT;
    }
    if (((imu_flags & IMU_DATA_REQUIRED) == IMU_DATA_REQUIRED) &&
        IMU_DataIsFresh(HAL_GetTick(), IMU_FRESHNESS_MS)) {
        state.status_flags |= STATE_IMU_VALID;
    }
    if (ready_flags == 0x0FFFU) {
        state.status_flags |= STATE_ENCODERS_VALID;
    }
    if (Protocol_CommandIsFresh(HAL_GetTick(), JETSON_COMMAND_WATCHDOG_MS)) {
        state.status_flags |= STATE_COMMAND_FRESH;
    }

    return JetsonUsbCdc_SendState(&state);
}

void JetsonRobotBridge_OnUsbReceive(uint8_t *data, uint32_t length)
{
    JetsonUsbCdc_OnReceive(data, length, HAL_GetTick());
}
