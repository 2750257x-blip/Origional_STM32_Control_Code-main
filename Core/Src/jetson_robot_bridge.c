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

/* ---- 反馈前向预测滤波 ---- */
#define SMOOTHING_ALPHA         0.0f     /* 0~1: 越大越跟随预测值 */
#define FRAME_ANGLE_LIMIT       0.5f     /* 反馈偏离目标的最大单帧变化量 (rad) */
#define MAX_PREDICT_DT_MS       50U      /* 预测dt上限 (ms)，防止卡顿瞬间跳变 */

/* ---- 前倾 20° + 反馈冻结 ---- */
#define LEAN_ANGLE_RAD   -0.3491f    /* 20° = 0.3491 rad，模型坐标系，叠加在站立姿态的髋 pitch 上 */
#define LEAN_RAMP_RATE   2.0f       /* rad/s：20° 约 290 ms 爬到位，避免站立时阶跃冲击 */

static float    lean_target;        /* 0 或 LEAN_ANGLE_RAD */
static float    lean_applied;       /* 带斜率地跟随 lean_target */
static uint32_t lean_last_ms;


/* ---- roll 关节偏置（补偿装配误差）----
 * 只在电机侧生效，上位机不知道：Nano 照常下发无偏置的目标，
 * motor_direction_target() 在发给电机之前加上去，
 * remove_roll_bias() 在反馈读数那里再减掉，一来一回正好抵消，
 * 所以 Nano 看到的仍然是自己下发的模型角度（只减一次，不会叠两次）。
 * 注意两者都作用在"符号翻转之后"的电机坐标系，roll 关节不在翻转名单里
 * （sign 恒为 +1），这才保证加减是在同一个坐标系里。若哪天把 roll 关节
 * 加进翻转名单（{0,4,6,10}），这里会变成同向叠加，必须一起改。
 * 左右腿是 mirror 装配，所以偏置反号：左腿 +bias，右腿 −bias。
 * 单位 rad（0.0175 rad ≈ 1°），可正可负，0 表示不加。
 * 直接改这两个量（ST-Link Live Expressions / 调试器），下一帧生效。 */
volatile float g_bias_hip_roll   = -0.03f;   /* 髋 roll：左 joint 1，右 joint 7 */
volatile float g_bias_ankle_roll = -0.03f;   /* 踝 roll：左 joint 5，右 joint 11 */

/* 冻结快照：前倾指令生效前抓的那一帧，冻结期间原样回放给 Nano */
static float frozen_joint_position[PROTOCOL_NUM_JOINTS];
static float frozen_joint_velocity[PROTOCOL_NUM_JOINTS];
static float frozen_imu[10];        /* accel(3) + gyro(3) + quat(4) */

/* Nano 下发的目标位置，用作平滑参考（无滞后、无噪声） */
static float target_position[PROTOCOL_NUM_JOINTS];
static uint32_t last_send_tick_ms;

volatile float g_debug_motor_target[PROTOCOL_NUM_JOINTS];
volatile uint8_t g_debug_jetson_control_active;
volatile uint32_t g_debug_watchdog_trip_count;
volatile uint32_t g_debug_invalid_command_count;
volatile float   g_debug_pose_lean_applied;
volatile uint8_t g_debug_feedback_freeze;

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

/* 前倾角按斜率爬向目标，避免站立状态下力矩瞬间打满 */
static float lean_offset_step(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - lean_last_ms) * 0.001f;
    float max_step;

    lean_last_ms = now;
    if (dt > 0.2f) dt = 0.2f;              /* 卡顿保护，防止一帧跨太大 */
    max_step = LEAN_RAMP_RATE * dt;

    if (lean_applied < lean_target) {
        lean_applied += max_step;
        if (lean_applied > lean_target) lean_applied = lean_target;
    } else if (lean_applied > lean_target) {
        lean_applied -= max_step;
        if (lean_applied < lean_target) lean_applied = lean_target;
    }
    g_debug_pose_lean_applied = lean_applied;
    return lean_applied;
}

static float motor_direction_target(uint8_t joint_index, float model_target)
{
    float target = model_target;

    /* These four joint directions are reversed between the URDF and motors. */
    if ((joint_index == 0U) || (joint_index == 4U) ||
        (joint_index == 6U) || (joint_index == 10U)) {
        target = -target;
    }
    /* 前倾：左右髋 pitch 镜像装配，偏置在电机坐标系里反号——
     * 左(0) +20°、右(6) −20°，其余关节不动 */
    if (joint_index == 0U) {
        target += lean_applied;
    } else if (joint_index == 6U) {
        target -= lean_applied;
    }
        /* roll 偏置：左右 mirror，反号 */
    if (joint_index == 1U) {            /* 左髋 roll */
        target += g_bias_hip_roll;
    } else if (joint_index == 7U) {     /* 右髋 roll */
        target -= g_bias_hip_roll;
    } else if (joint_index == 5U) {     /* 左踝 roll */
        target += g_bias_ankle_roll;
    } else if (joint_index == 11U) {    /* 右踝 roll */
        target -= g_bias_ankle_roll;
    }
    return target;
}

/* motor_direction_target() 里 roll 偏置的逆变换：电机读回来的角度带着那个偏置，
 * 减掉它，上位机看到的才是自己下发的模型角度。速度不受影响（偏置是常数）。 */
static float remove_roll_bias(uint8_t joint_index, float value)
{
    if (joint_index == 1U) {            /* 左髋 roll */
        return value - g_bias_hip_roll;
    } else if (joint_index == 7U) {     /* 右髋 roll */
        return value + g_bias_hip_roll;
    } else if (joint_index == 5U) {     /* 左踝 roll */
        return value - g_bias_ankle_roll;
    } else if (joint_index == 11U) {    /* 右踝 roll */
        return value + g_bias_ankle_roll;
    }
    return value;
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

static void apply_position_targets(const RobotCommandPayload *command)
{
    static const float base_kp[6] = {
        40.0f, 40.0f, 40.0f, 40.0f, 40.0f, 40.0f
    };
    static const float base_kd[6] = {
        2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f
    };
    static const uint8_t left_motor_id[6] = {
        l_leg_pitch, l_leg_roll, l_leg_yaw,
        l_knee_pitch, l_ankle_pitch, l_ankle_roll
    };
    static const uint8_t right_motor_id[6] = {
        r_leg_pitch, r_leg_roll, r_leg_yaw,
        r_knee_pitch, r_ankle_pitch, r_ankle_roll
    };
    // float kp_scale = limit_gain_scale(command->kp_scale);
    // float kd_scale = limit_gain_scale(command->kd_scale);
    float kp_scale = 1.0f;
    float kd_scale = 1.0f;
    uint8_t index;

    (void)lean_offset_step();

    for (index = 0U; index < 6U; ++index) {
        float target = motor_direction_target(index, command->joint_target[index]);
        //  if(index == 0U) {
        //     target = target - 0.08f;
        // }
        g_debug_motor_target[index] = target;
        Motor_limitCtrl_float(
            &hfdcan2,
            left_motor_id[index],
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
        g_debug_motor_target[protocol_index] = target;
        Motor_limitCtrl_float(
            &hfdcan1,
            right_motor_id[index],
            target,
            0.0f,
            base_kp[index] * kp_add * kp_scale,
            base_kd[index] * kd_add * kd_scale);
    }
    system_control_cycle ++;
}

void JetsonRobotBridge_CaptureFrozenState(void)
{
    uint8_t index;

    __disable_irq();
    for (index = 0U; index < PROTOCOL_NUM_JOINTS; ++index) {
        float sign = ((index == 0U) || (index == 4U) ||
                      (index == 6U) || (index == 10U)) ? -1.0f : 1.0f;
        frozen_joint_position[index] =
            remove_roll_bias(index, sign * MotorIMU_Packet_float[index * 2U]);
        frozen_joint_velocity[index] = sign * MotorIMU_Packet_float[index * 2U + 1U];
    }
    memcpy(frozen_imu, &MotorIMU_Packet_float[24], sizeof(frozen_imu));
    __enable_irq();
}

void JetsonRobotBridge_SetLean(uint8_t active)
{
    lean_last_ms = HAL_GetTick();
    lean_target = (active != 0U) ? LEAN_ANGLE_RAD : 0.0f;
}

/* 斜坡走完了没有。lean_offset_step() 到位时是直接赋值的，所以这里能比。
 * 没在做前倾/恢复时两边都是 0，恒为真。 */
uint8_t JetsonRobotBridge_LeanSettled(void)
{
    return (uint8_t)(lean_applied == lean_target);
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
    lean_target = 0.0f;
    lean_applied = 0.0f;
    lean_last_ms = HAL_GetTick();
    g_debug_pose_lean_applied = 0.0f;
    g_debug_feedback_freeze = 0U;
    memset(frozen_joint_position, 0, sizeof(frozen_joint_position));
    memset(frozen_joint_velocity, 0, sizeof(frozen_joint_velocity));
    memset(frozen_imu, 0, sizeof(frozen_imu));
    last_send_tick_ms = HAL_GetTick();
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

       /* 原始反馈（已转模型坐标系，并减掉 roll 偏置还原成上位机坐标系） */
        float raw_pos = remove_roll_bias(index, sign * feedback[index * 2U]);
        float raw_vel = sign * feedback[index * 2U + 1U];

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
    memcpy(state.accel_m_s2, &feedback[24], sizeof(state.accel_m_s2));
    memcpy(state.gyro_rad_s, &feedback[27], sizeof(state.gyro_rad_s));
    memcpy(state.orientation_wxyz, &feedback[30], sizeof(state.orientation_wxyz));

    /* 冻结期间回放前倾前的快照：上位机拿假数据做识别，电机侧闭环不受影响。
     * timestamp_us 不冻——它是"新一帧"的单调时钟，冻掉会让上位机误判过期。 */
    if (g_debug_feedback_freeze != 0U) {
        memcpy(state.joint_position, frozen_joint_position, sizeof(state.joint_position));
        memcpy(state.joint_velocity, frozen_joint_velocity, sizeof(state.joint_velocity));
        memcpy(state.accel_m_s2, &frozen_imu[0], sizeof(state.accel_m_s2));
        memcpy(state.gyro_rad_s, &frozen_imu[3], sizeof(state.gyro_rad_s));
        memcpy(state.orientation_wxyz, &frozen_imu[6], sizeof(state.orientation_wxyz));
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
