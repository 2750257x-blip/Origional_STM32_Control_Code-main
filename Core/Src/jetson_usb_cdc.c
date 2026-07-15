#include "jetson_usb_cdc.h"

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

/* Defined by USB_DEVICE/App/usb_device.c. */
extern USBD_HandleTypeDef hUsbDeviceHS;

#define STATE_FRAME_SIZE \
    (sizeof(ProtocolHeader) + sizeof(RobotStatePayload) + 2U)

static uint8_t state_frame[STATE_FRAME_SIZE];

void JetsonUsbCdc_OnReceive(
    const uint8_t *data,
    uint32_t length,
    uint32_t now_ms)
{
    while (length > 0U) {
        uint16_t chunk = (length > 65535U) ? 65535U : (uint16_t)length;
        Protocol_RxBytes(data, chunk, now_ms);
        data += chunk;
        length -= chunk;
    }
}

uint8_t JetsonUsbCdc_SendState(const RobotStatePayload *state)
{
    USBD_CDC_HandleTypeDef *cdc =
        (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    uint16_t length;

    if ((cdc == NULL) || (cdc->TxState != 0U)) {
        return USBD_BUSY;
    }

    length = Protocol_EncodeState(state, state_frame, sizeof(state_frame));
    if (length == 0U) {
        return USBD_FAIL;
    }

    return CDC_Transmit_HS(state_frame, length);
}
