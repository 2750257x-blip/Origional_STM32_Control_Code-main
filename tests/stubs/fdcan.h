#ifndef TEST_FDCAN_H
#define TEST_FDCAN_H
#include <stdint.h>
typedef struct { uint32_t PSR, CCCR, TXBRP; } Registers;
typedef struct { Registers *Instance; } FDCAN_HandleTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
    uint32_t BitRateSwitch, FDFormat, TxEventFifoControl;
} FDCAN_TxHeaderTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
#define FDCAN_PSR_BO 0x80U
#define FDCAN_CCCR_INIT 1U
#define CLEAR_BIT(reg, mask) ((reg) &= ~(mask))
#define FDCAN_EXTENDED_ID 1
#define FDCAN_DATA_FRAME 0
#define FDCAN_ESI_ACTIVE 0
#define FDCAN_BRS_OFF 0
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_NO_TX_EVENTS 0
#define HAL_OK 0
uint32_t HAL_GetTick(void);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, uint8_t *);
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
uint8_t FDCAN_Send_ExtFrame(FDCAN_HandleTypeDef *, uint32_t, uint8_t *, uint8_t);
#endif
