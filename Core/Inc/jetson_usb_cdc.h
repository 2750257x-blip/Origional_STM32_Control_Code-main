#ifndef JETSON_USB_CDC_H
#define JETSON_USB_CDC_H

#include "jetson_protocol.h"
#include <stdint.h>

void JetsonUsbCdc_OnReceive(
    const uint8_t *data,
    uint32_t length,
    uint32_t now_ms);
uint8_t JetsonUsbCdc_SendState(const RobotStatePayload *state);

#endif
