#include <stdint.h>
#ifdef HAVE_ACTION_GOTO
#include "action_goto.h"
static ActionGoto goto_motion;
#endif

typedef struct { uint32_t unused; } TIM_HandleTypeDef;
typedef struct { uint32_t unused; } FDCAN_HandleTypeDef;
static TIM_HandleTypeDef htim4;
static FDCAN_HandleTypeDef hfdcan1, hfdcan2;
static float leg_control[12];
static volatile uint8_t Goto_ready, Goto_number;
static volatile float Goto_walklength[12];
static const float kp_add = 1.2f, kd_add = 1.5f;
static uint32_t tick_ms;
static uint32_t timer_start_count, motor_send_count;
static uint32_t goto_last_send_ms;
static volatile uint8_t g_debug_local_leg_active;
static uint8_t local_watchdog_required;
static volatile uint8_t g_debug_jetson_control_active;
enum { r_leg_pitch, r_leg_roll, r_leg_yaw, r_knee_pitch,
       r_ankle_pitch, r_ankle_roll, l_leg_pitch, l_leg_roll,
       l_leg_yaw, l_knee_pitch, l_ankle_pitch, l_ankle_roll };
#define TIM_FLAG_UPDATE 1
#define __HAL_TIM_SET_COUNTER(timer, value) ((void)0)
#define __HAL_TIM_CLEAR_FLAG(timer, flag) ((void)0)
static void HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *timer)
{ (void)timer; ++timer_start_count; }
static void HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *timer) { (void)timer; }
static uint32_t HAL_GetTick(void) { return tick_ms; }
static void EL05_Motor_Ctrl(FDCAN_HandleTypeDef *bus, uint8_t motor,
                          float torque, float position, float speed,
                          float kp, float kd)
{
    (void)bus; (void)motor; (void)torque; (void)position;
    (void)speed; (void)kp; (void)kd; ++motor_send_count;
}

/* The runner inserts the current, unmodified main.c function here. */
/* INSERT_ACTION_GOTO */

uint32_t test_wrapper(void)
{
    tick_ms = 123U;
    Action_Goto(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f,
                -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, 50);
    return 1U;
}
