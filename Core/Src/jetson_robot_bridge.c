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
#define MAX_GAIN_SCALE             2.0f

static uint32_t last_applied_command_count;

/* ---- 反馈前向预测滤波 ---- */
#define SMOOTHING_ALPHA         0.0f     /* 0~1: 越大越跟随预测值 */
#define FRAME_ANGLE_LIMIT       0.5f     /* 反馈偏离目标的最大单帧变化 (rad) */
#define MAX_PREDICT_DT_MS       50U      /* 预测Δt上限 (ms)，防止卡顿时跳变 */

/* Nano 下发的目标位置，用作平滑参考（无滞后、无噪声） */
static float target_position[PROTOCOL_NUM_JOINTS];
static uint32_t last_send_tick_ms;

volatile float g_debug_motor_target[PROTOCOL_NUM_JOINTS];
volatile uint8_t g_debug_jetson_control_active;
volatile uint32_t g_debug_watchdog_trip_count;
volatile uint32_t g_debug_invalid_command_count;

extern volatile uint32_t system_control_cycle;
 
const float kp_add = 1.5f;
const float kd_add = 2.0f;
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

    for (index = 0U; index < 6U; ++index) {
        float target = motor_direction_target(index, command->joint_target[index]);
        // if(index == 1U) {
        //     target = target + 0.05f;
        // }
        //  if(index == 0U) {
        //     target = target - 0.15f;
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
        // if(index == 1U) {
        //     target = target - 0.05f;
        // }
        // if(index == 0U) {
        //     target = target +  0.15f;
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
    float feedback[30];
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

        /* 原始反馈（已转模型坐标系） */
        float raw_pos = sign * feedback[index * 2U];
        float raw_vel = sign * feedback[index * 2U + 1U];

        /* 1. 速度外推 — 补偿1帧滞后：p_pred = p_raw + v * Δt */
        float predicted = raw_pos + raw_vel * delta_t;

        /* 2. 向目标位置平滑 — 以外推为主，以目标为参考抑制噪声 */
        // float corrected = SMOOTHING_ALPHA * predicted
        //                 + (1.0f - SMOOTHING_ALPHA) * target_position[index];
        float corrected = SMOOTHING_ALPHA * predicted
                        + (1.0f - SMOOTHING_ALPHA) * raw_pos;


        /* 3. 单帧限幅 — 反馈偏离目标不超过限制值，过滤跳变 */
        float delta = corrected - target_position[index];
        if (delta > FRAME_ANGLE_LIMIT)  delta = FRAME_ANGLE_LIMIT;
        if (delta < -FRAME_ANGLE_LIMIT) delta = -FRAME_ANGLE_LIMIT;
        corrected = target_position[index] + delta;

        state.joint_position[index] = corrected;
        state.joint_velocity[index] = raw_vel;
    }
    memcpy(state.accel_m_s2, &feedback[24], sizeof(state.accel_m_s2));
    memcpy(state.gyro_rad_s, &feedback[27], sizeof(state.gyro_rad_s));

    if (mode_flags == 0x0FFFU) {
        state.status_flags |= STATE_MOTORS_ENABLED;
    }
    if (fault_flags != 0U) {
        state.status_flags |= STATE_FAULT;
    }
    if ((imu_flags & 0x03U) == 0x03U) {
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
