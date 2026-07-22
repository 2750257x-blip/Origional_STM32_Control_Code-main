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

#define COMMAND_ENABLE              (1UL << 0)
#define COMMAND_ESTOP               (1UL << 1)
#define COMMAND_CLEAR_FAULT         (1UL << 2)

#define STATE_MOTORS_ENABLED        (1UL << 0)
#define STATE_FAULT                 (1UL << 1)
#define STATE_IMU_VALID             (1UL << 2)
#define STATE_ENCODERS_VALID        (1UL << 3)
#define STATE_COMMAND_FRESH         (1UL << 4)

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
} RobotStatePayload;

typedef struct {
    uint32_t timestamp_us;
    float joint_target[PROTOCOL_NUM_JOINTS];
    float kp_scale;
    float kd_scale;
    uint32_t command_flags;
} RobotCommandPayload;

/* Debug variables readable through ST-Link/SWD and CubeIDE Live Expressions. */
extern volatile RobotCommandPayload g_debug_latest_command;
extern volatile uint16_t g_debug_command_sequence;
extern volatile uint32_t g_debug_command_count;
extern volatile uint32_t g_debug_command_received_ms;
extern volatile uint32_t g_debug_crc_error_count;

#pragma pack(pop)

_Static_assert(sizeof(ProtocolHeader) == 8U, "ProtocolHeader wire size must be 8 bytes");
_Static_assert(sizeof(RobotStatePayload) == 144U, "RobotStatePayload wire size must be 144 bytes");
_Static_assert(sizeof(RobotCommandPayload) == 64U, "RobotCommandPayload wire size must be 64 bytes");

void Protocol_Init(void);
void Protocol_RxBytes(const uint8_t *data, uint16_t length, uint32_t now_ms);
bool Protocol_GetFreshCommand(uint32_t now_ms, uint32_t maximum_age_ms, RobotCommandPayload *output);
bool Protocol_CommandIsFresh(uint32_t now_ms, uint32_t maximum_age_ms);
uint16_t Protocol_EncodeState(const RobotStatePayload *state, uint8_t *output, uint16_t capacity);
uint32_t Protocol_GetCrcErrorCount(void);

#endif
