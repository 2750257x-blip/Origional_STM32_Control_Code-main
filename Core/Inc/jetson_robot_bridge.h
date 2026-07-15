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

/* Values after URDF-to-motor direction conversion, visible over ST-Link. */
extern volatile float g_debug_motor_target[12];
extern volatile uint8_t g_debug_jetson_control_active;
extern volatile uint32_t g_debug_watchdog_trip_count;
extern volatile uint32_t g_debug_invalid_command_count;

#endif
