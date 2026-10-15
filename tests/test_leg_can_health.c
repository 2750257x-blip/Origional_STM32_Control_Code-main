/* Host tests against the production transport module; no physical CAN device.
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/stubs -ICore/Inc
 *     tests/test_leg_can_health.c Core/Src/leg_can_health.c -o test_leg_can
 */
#include "leg_can_health.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static Registers regs[3];
FDCAN_HandleTypeDef hfdcan1 = {&regs[0]}, hfdcan2 = {&regs[1]}, hfdcan3 = {&regs[2]};
static uint32_t now = 1000, free_level = 6, free_after, polls, sends, aborts;
static int fail_enqueue;
uint32_t HAL_GetTick(void) { return now++; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *bus)
{
    (void)bus;
    if (free_after && ++polls >= free_after) return 1;
    return free_level;
}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *bus, const FDCAN_TxHeaderTypeDef *header, uint8_t *data)
{
    (void)bus; (void)data;
    assert(header->DataLength == 8);
    ++sends;
    return fail_enqueue || (!free_level && (!free_after || polls < free_after));
}
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *bus, uint32_t pending)
{
    assert(pending != 0);
    ++aborts;
    bus->Instance->TXBRP &= ~pending;
    return HAL_OK;
}
int main(void)
{
    uint8_t data[8] = {0};
    assert(LegCan_FreshMask(now) == 0);
    for (unsigned i = 0; i < 12; ++i) { g_leg_rx_count[i] = 1; g_leg_rx_ms[i] = now; }
    assert(LegCan_FreshMask(now) == 0xfff);
    for (unsigned i = 0; i < 6; ++i) g_leg_rx_ms[i] -= 101;
    assert(LegCan_FreshMask(now) == 0xfc0); /* left stale, right fresh */
    g_leg_rx_ms[0] = 0xfffffff0U;
    assert(LegCan_FreshMask(0x10U) & 1U); /* tick rollover */

    assert(FDCAN_Send_ExtFrame(&hfdcan1, 123, data, 8) == 0);
    assert(g_leg_tx_failures[0] == 0);
    free_level = 0;
    uint32_t before = now;
    assert(FDCAN_Send_ExtFrame(&hfdcan2, 123, data, 8) != 0);
    assert(now - before < 10); /* bounded wait; no indefinite blocked main loop */
    assert(g_leg_tx_failures[1] == 1);
    assert(!LegCan_Healthy(now));
    free_after = 2; polls = 0;
    assert(FDCAN_Send_ExtFrame(&hfdcan2, 123, data, 8) == 0);
    assert(g_leg_tx_failures[1] == 1); /* transient FIFO pressure retried */
    now += 101;
    assert(LegCan_Healthy(now));

    regs[0].TXBRP = 3; regs[1].TXBRP = 12;
    LegCan_BeginBatch();
    assert(regs[0].TXBRP == 0 && regs[1].TXBRP == 0 && aborts == 2);
    regs[1].PSR = FDCAN_PSR_BO; regs[1].CCCR = FDCAN_CCCR_INIT; regs[1].TXBRP = 63;
    assert(!LegCan_Healthy(now));
    LegCan_Service();
    assert(regs[1].TXBRP == 0 && !(regs[1].CCCR & FDCAN_CCCR_INIT));
    assert(g_leg_recoveries[1] == 1);
    LegCan_Service();
    assert(g_leg_recoveries[1] == 1); /* no recovery storm */
    now += 251;
    LegCan_Service();
    assert(g_leg_recoveries[1] == 2);
    regs[1].PSR = 0;
    assert(LegCan_Healthy(now));
    puts("PASS: feedback freshness, wrap, bounded FIFO retry, failure reporting, stale TX cancellation, bus-off recovery");
    return 0;
}
