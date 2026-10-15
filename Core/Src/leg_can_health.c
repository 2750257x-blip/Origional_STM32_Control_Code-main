#include "leg_can_health.h"

volatile uint32_t g_leg_rx_ms[12];
volatile uint32_t g_leg_rx_count[12];
volatile uint32_t g_leg_tx_failures[2];
volatile uint32_t g_leg_recoveries[2];
static uint32_t failure_ms[2];
static uint32_t recovery_ms[2];
static uint8_t recovery_started[2];

uint16_t LegCan_FreshMask(uint32_t now)
{
    uint16_t mask = 0U;
    for (uint8_t i = 0; i < 12; ++i) {
        if (g_leg_rx_count[i] != 0U && (uint32_t)(now - g_leg_rx_ms[i]) <= LEG_FEEDBACK_TIMEOUT_MS)
            mask |= (uint16_t)(1U << i);
    }
    return mask;
}

bool LegCan_Healthy(uint32_t now)
{
    FDCAN_HandleTypeDef *buses[2] = {&hfdcan1, &hfdcan2};
    for (uint8_t b = 0; b < 2; ++b) {
        if ((buses[b]->Instance->PSR & FDCAN_PSR_BO) != 0U ||
            (buses[b]->Instance->CCCR & FDCAN_CCCR_INIT) != 0U ||
            (g_leg_tx_failures[b] != 0U && (uint32_t)(now - failure_ms[b]) < 100U)) return false;
    }
    return true;
}

void LegCan_Service(void)
{
    FDCAN_HandleTypeDef *buses[2] = {&hfdcan1, &hfdcan2};
    uint32_t now = HAL_GetTick();
    for (uint8_t b = 0; b < 2; ++b) {
        FDCAN_HandleTypeDef *bus = buses[b];
        if ((bus->Instance->PSR & FDCAN_PSR_BO) == 0U) continue;
        if (!recovery_started[b] || (uint32_t)(now - recovery_ms[b]) >= 250U) {
            /* Cancel obsolete PD/position requests BEFORE recovery. FDCAN does
             * not implement bxCAN's automatic bus-off recovery: clear INIT to
             * start the hardware's 129 x 11 recessive-bit recovery sequence. */
            uint32_t pending = bus->Instance->TXBRP;
            if (pending != 0U) (void)HAL_FDCAN_AbortTxRequest(bus, pending);
            CLEAR_BIT(bus->Instance->CCCR, FDCAN_CCCR_INIT);
            recovery_ms[b] = now;
            recovery_started[b] = 1U;
            ++g_leg_recoveries[b];
        }
    }
}

void LegCan_BeginBatch(void)
{
    FDCAN_HandleTypeDef *buses[2] = {&hfdcan1, &hfdcan2};
    /* A normal six-frame batch drains well before the next 20 ms COMMAND.
     * Never let an old held-position command sit ahead of a new zero-PD one. */
    for (uint8_t b = 0; b < 2; ++b) {
        uint32_t pending = buses[b]->Instance->TXBRP;
        if (pending != 0U) (void)HAL_FDCAN_AbortTxRequest(buses[b], pending);
    }
}

uint8_t FDCAN_Send_ExtFrame(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef header = {0};
    int b = bus == &hfdcan1 ? 0 : bus == &hfdcan2 ? 1 : -1;
    if (len < 1U || len > 8U) return 1U;
    header.Identifier = id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = len; /* This project's HAL uses unshifted DLC values. */
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    if (b >= 0) {
        uint32_t start = HAL_GetTick();
        uint32_t spins = 0;
        /* Bounded wait handles ENABLE + position bursts without dropping the
         * latter when the six-entry hardware FIFO is still occupied. */
        while (HAL_FDCAN_GetTxFifoFreeLevel(bus) == 0U &&
               (uint32_t)(HAL_GetTick() - start) < 2U && ++spins < 100000U) { }
    }
    if ((b < 0 || (bus->Instance->PSR & FDCAN_PSR_BO) == 0U) &&
        HAL_FDCAN_AddMessageToTxFifoQ(bus, &header, data) == HAL_OK) return 0U;
    if (b >= 0) {
        ++g_leg_tx_failures[b];
        failure_ms[b] = HAL_GetTick();
    }
    return 1U;
}
