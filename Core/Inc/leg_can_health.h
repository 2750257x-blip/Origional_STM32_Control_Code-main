#ifndef LEG_CAN_HEALTH_H
#define LEG_CAN_HEALTH_H
#include <fdcan.h>
#include <stdbool.h>

#define LEG_FEEDBACK_TIMEOUT_MS 100U
/* All joint arrays use protocol order: left six, then right six. */
extern volatile uint32_t g_leg_rx_ms[12];
extern volatile uint32_t g_leg_rx_count[12];
extern volatile uint32_t g_leg_tx_failures[2]; /* CAN1=right, CAN2=left */
extern volatile uint32_t g_leg_recoveries[2];
void LegCan_Service(void);
void LegCan_BeginBatch(void);
bool LegCan_Healthy(uint32_t now);
uint16_t LegCan_FreshMask(uint32_t now);
#endif
