#ifndef JETSON_PROTOCOL_H
#define JETSON_PROTOCOL_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define PROTOCOL_MAGIC              0xA55AU
#define PROTOCOL_VERSION            2U
#define PROTOCOL_NUM_JOINTS         12U
#define PROTOCOL_MSG_STATE          1U
#define PROTOCOL_MSG_COMMAND        2U
#define PROTOCOL_MSG_ACTION_REQUEST 3U
#define PROTOCOL_MSG_ACTION_STATUS  4U
#define PROTOCOL_MSG_STARTUP_CONTROL 5U

/* One-byte STARTUP_CONTROL payload; independent of motor COMMAND/watchdog. */
#define STARTUP_ARM                 (1U << 0)
#define STARTUP_CARD_READY          (1U << 1)

#define ACTION_STATUS_ACCEPTED      1U
#define ACTION_STATUS_DONE          2U
#define ACTION_STATUS_BUSY          3U
#define ACTION_STATUS_INVALID       4U
#define ACTION_STATUS_FAILED        5U

/* action_id 空间：7/8 是姿态事件，不参与 Robot_State_Machine。
 * 1-6 是六种图形 —— 其中 3/4（正方形/菱形）走 Nano 自己的抬腿策略，
 * 从来不下发到这里，所以线上 3/4 本来就是空的（但不用它，免得和图形号混淆）。
 * 上位机那边 Jetson_orin_nano_code/humanoid_jetson_deploy/protocol.py 里
 * 就是 ACTION_CARD_TILT = 7 / ACTION_CARD_RESTORE = 8，两边必须一致。 */
#define ACTION_ID_LEAN              7U
#define ACTION_ID_RESTORE           8U

#define COMMAND_ENABLE              (1UL << 0)
#define COMMAND_ESTOP               (1UL << 1)
#define COMMAND_CLEAR_FAULT         (1UL << 2)

#define STATE_MOTORS_ENABLED        (1UL << 0)
#define STATE_FAULT                 (1UL << 1)
#define STATE_IMU_VALID             (1UL << 2)
#define STATE_ENCODERS_VALID        (1UL << 3)
#define STATE_COMMAND_FRESH         (1UL << 4)
#define STATE_START_BUTTON          (1UL << 5)
#define STATE_STARTUP_ACTIVE        (1UL << 6)

#pragma pack(push, 1)
typedef struct {
    uint16_t magic;
    uint8_t version;
    uint8_t message_type;
    uint16_t payload_length;
    uint16_t sequence;
} ProtocolHeader;

typedef struct {
    uint32_t timestamp_us;
    float joint_position[PROTOCOL_NUM_JOINTS];
    float joint_velocity[PROTOCOL_NUM_JOINTS];
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float orientation_wxyz[4];
    uint32_t status_flags;
    /* CRC-valid COMMAND frames received, including commands not applied. */
    uint32_t command_rx_count;
    /* Calls that submitted all 12 leg targets to the motor CAN buses. */
    uint32_t system_control_cycle;
} RobotStatePayload;

typedef struct {
    uint32_t timestamp_us;
    float joint_target[PROTOCOL_NUM_JOINTS];
    /* Absolute gains, in the same left/right joint order as joint_target. */
    float kp[PROTOCOL_NUM_JOINTS];
    float kd[PROTOCOL_NUM_JOINTS];
    uint32_t command_flags;
} RobotCommandPayload;

typedef struct {
    uint32_t event_id;
    uint8_t action_id;
} ActionRequestPayload;

typedef struct {
    uint32_t event_id;
    uint8_t action_id;
    uint8_t status;
} ActionStatusPayload;

/* Debug variables readable through ST-Link/SWD and CubeIDE Live Expressions. */
extern volatile RobotCommandPayload g_debug_latest_command;
extern volatile uint16_t g_debug_command_sequence;
extern volatile uint32_t g_debug_command_count;
extern volatile uint32_t g_debug_command_received_ms;
extern volatile uint32_t g_debug_crc_error_count;
/* Receipt tick of the startup control most recently taken by the main loop. */
extern volatile uint32_t g_debug_startup_received_ms;

#pragma pack(pop)

_Static_assert(sizeof(ProtocolHeader) == 8U, "ProtocolHeader wire size must be 8 bytes");
_Static_assert(sizeof(RobotStatePayload) == 152U, "RobotStatePayload wire size must be 152 bytes");
_Static_assert(sizeof(RobotCommandPayload) == 152U, "RobotCommandPayload wire size must be 152 bytes");
_Static_assert(sizeof(ActionRequestPayload) == 5U, "ActionRequestPayload wire size must be 5 bytes");
_Static_assert(sizeof(ActionStatusPayload) == 6U, "ActionStatusPayload wire size must be 6 bytes");

void Protocol_Init(void);
void Protocol_RxBytes(const uint8_t *data, uint16_t length, uint32_t now_ms);
bool Protocol_GetFreshCommand(uint32_t now_ms, uint32_t maximum_age_ms, RobotCommandPayload *output);
bool Protocol_CommandIsFresh(uint32_t now_ms, uint32_t maximum_age_ms);
bool Protocol_TakeActionRequest(ActionRequestPayload *output);
bool Protocol_TakeStartupControl(uint8_t *flags);
uint16_t Protocol_EncodeState(const RobotStatePayload *state, uint8_t *output, uint16_t capacity);
uint16_t Protocol_EncodeActionStatus(const ActionStatusPayload *status, uint8_t *output, uint16_t capacity);
uint32_t Protocol_GetCrcErrorCount(void);

#endif
