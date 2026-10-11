#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "action_goto.h"
typedef float float_t;
/* INSERT_PROTOCOL_HEADER */
/* INSERT_ROBOT_ENUM */
/* INSERT_MOTOR_DEFINES */
#define ACTION_TICK_MS 10U
#define JETSON_COMMAND_WATCHDOG_MS 100U
#define USBD_OK 0U
#define GPIOA ((void *)1)
#define GPIO_PIN_1 1U
#define GPIO_PIN_3 3U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define TIM_CHANNEL_1 1U
#define TIM_CHANNEL_2 2U
#define TIM_CHANNEL_3 3U
#define DATA_OUTPUT_SELECTION 0U
#define ACTION_IDLE 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } FDCAN_HandleTypeDef;
static TIM_HandleTypeDef htim1;
static FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static uint8_t motor_status_buf[48];
static uint16_t motor_status_ready;
static uint8_t action_state;
static volatile uint8_t g_debug_jetson_control_active;
static volatile uint8_t g_debug_feedback_freeze, g_debug_startup_active;
volatile uint32_t g_debug_command_count;
static uint32_t g_debug_watchdog_trip_count, g_debug_estop_trip_count;
static uint32_t g_debug_invalid_command_count;
static uint32_t last_applied_command_count;
static float target_position[12];
static volatile float g_debug_motor_target[12];
static RobotCommandPayload incoming_command;
static uint32_t tick_ms, host_apply_count, telemetry_count, process_count;
static uint32_t motor_send_count, motor_stop_count, reset_count, status_count;
static uint32_t status_attempts;
static uint8_t status_tx_busy;
static uint8_t host_fresh = 1U, command_valid = 1U;
static ActionStatusPayload last_status;

/* INSERT_MAIN_GLOBALS */

void *memcpy(void *destination, const void *source, size_t size)
{
    uint8_t *to = destination;
    const uint8_t *from = source;
    while (size-- != 0U) *to++ = *from++;
    return destination;
}
static uint32_t HAL_GetTick(void) { return tick_ms; }
static void HAL_Delay(uint32_t ms) { (void)ms; }
static void HAL_GPIO_WritePin(void *port, uint32_t pin, uint32_t value)
{ (void)port; (void)pin; (void)value; }
static void HAL_NVIC_SystemReset(void) { ++reset_count; }
static void Servo_SetAngle(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t angle)
{ (void)timer; (void)channel; (void)angle; }
static void EL05_Motor_Ctrl(FDCAN_HandleTypeDef *bus, uint8_t motor,
                          float torque, float position, float speed, float kp, float kd)
{
    (void)bus; (void)motor; (void)torque; (void)position;
    (void)speed; (void)kp; (void)kd; ++motor_send_count;
}
static void EL05_Motor_Stop(FDCAN_HandleTypeDef *bus, uint8_t motor)
{ (void)bus; (void)motor; ++motor_stop_count; }
static void stop_all_motors(void) { motor_stop_count += 12U; }
static void motor_enable(void) { }
bool Protocol_CommandIsFresh(uint32_t now_ms, uint32_t maximum_age_ms)
{ (void)now_ms; (void)maximum_age_ms; return host_fresh != 0U; }
bool Protocol_GetFreshCommand(uint32_t now_ms, uint32_t maximum_age_ms,
                              RobotCommandPayload *output)
{
    (void)now_ms; (void)maximum_age_ms; ++process_count;
    memcpy(output, &incoming_command, sizeof(*output));
    return host_fresh != 0U;
}
bool Protocol_TakeActionRequest(ActionRequestPayload *output)
{ (void)output; return false; }
static uint8_t JetsonUsbCdc_SendActionStatus(const ActionStatusPayload *status)
{
    ++status_attempts;
    if (status_tx_busy != 0U) return 1U;
    last_status = *status; ++status_count; return USBD_OK;
}
static uint8_t JetsonRobotBridge_SendState(void) { ++telemetry_count; return USBD_OK; }
static uint8_t JetsonRobotBridge_LeanSettled(void) { return 1U; }
static void JetsonRobotBridge_CaptureFrozenState(void) { }
static void JetsonRobotBridge_SetLean(uint8_t active) { (void)active; }
static uint8_t command_targets_are_valid(const RobotCommandPayload *command)
{ (void)command; return command_valid; }
static void apply_position_targets(const RobotCommandPayload *command)
{ (void)command; ++host_apply_count; }
static void clear_startup(void) { g_debug_startup_active = 0U; }
static void imu_change_to_request(void) { }
static void imu_set_active_mode_delay(uint32_t ms) { (void)ms; }
static void imu_write_reg(uint32_t reg, uint32_t value) { (void)reg; (void)value; }
static void imu_save_parameters(void) { }
static void imu_change_to_active(void) { }
static void ROBOT_RHAND(void) { }
static void ROBOT_LHAND(void) { }
static void ROBOT_HEAD(void) { }
static void ROBOT_RESET(void) { }
static void ROBOT_BOTHH(void) { }
static void ROBOT_CROSS(void) { }
static void ROBOT_TEST_IMU(void) { }
static void ROBOT_TEST_UART(void) { }
static void ROBOT_TEST_INIT(void) { }

/* INSERT_PRODUCTION_FUNCTIONS */

uint32_t test_wrapper(void)
{
    g_debug_jetson_control_active = 1U;
    motor_status_ready = 0x0FFFU;
    incoming_command.command_flags = COMMAND_ENABLE;
    /* Near-zero encoder feedback is deliberately different from the stand pose. */
    for (uint32_t i = 0; i < 12; ++i) {
        motor_status_buf[i * 4] = 0x7fU;
        motor_status_buf[i * 4 + 1] = 0xffU;
    }
    return 1U;
}
uint32_t test_request(uint32_t event_id, uint32_t action_id)
{
    ActionRequestPayload request = {event_id, (uint8_t)action_id};
    Handle_Action_Request(&request);
    return pending_action_status.status;
}
uint32_t test_loop(uint32_t now_ms)
{
    tick_ms = now_ms;
    action_tick = now_ms / ACTION_TICK_MS;
    ++g_debug_command_count;
    ROBOT_Comms_Service();
    Action_Goto_Service();
    Robot_State_Machine();
    return active_action_status;
}
uint32_t test_fresh(uint32_t fresh) { host_fresh = (uint8_t)fresh; return fresh; }
uint32_t test_command_flags(uint32_t flags)
{ incoming_command.command_flags = flags; return flags; }
uint32_t test_status_busy(uint32_t busy) { status_tx_busy = (uint8_t)busy; return busy; }
uint32_t test_command_valid(uint32_t valid) { command_valid = (uint8_t)valid; return valid; }
uint32_t test_offline_action(uint32_t action_id)
{
    host_fresh = 0U;
    g_debug_jetson_control_active = 0U;
    robot_state = action_id == ACTION_ID_LEFT_LEG ? ROBOT_STATE_LLEG : ROBOT_STATE_RLEG;
    return robot_state;
}
uint32_t test_metric(uint32_t index)
{
    switch (index) {
    case 0: return robot_state;
    case 1: return g_debug_local_leg_active;
    case 2: return active_action_status;
    case 3: return last_status.status;
    case 4: return host_apply_count;
    case 5: return telemetry_count;
    case 6: return process_count;
    case 7: return motor_send_count;
    case 8: return motor_stop_count;
    case 9: return status_count;
    case 10: return goto_motion.busy;
    case 11: return action_step;
    case 12: return action_count_finished;
    case 13: return reset_count;
    case 14: return status_attempts;
    case 15: return system_control_cycle;
    case 16: return g_debug_jetson_control_active;
    default: return 0xffffffffU;
    }
}
uint32_t test_joint(uint32_t index)
{
    union { float number; uint32_t bits; } value;
    value.number = leg_control[index];
    return value.bits;
}
