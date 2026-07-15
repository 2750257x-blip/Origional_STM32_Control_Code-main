#include "jetson_protocol.h"

#include <string.h>

#define HEADER_SIZE             ((uint16_t)sizeof(ProtocolHeader))
#define CRC_SIZE                2U
#define MAX_PAYLOAD_SIZE        512U
#define MAX_FRAME_SIZE          (HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE)
#define MAGIC_BYTE_0            0x5AU
#define MAGIC_BYTE_1            0xA5U

static uint8_t rx_frame[MAX_FRAME_SIZE];
static uint16_t rx_length;
static uint16_t rx_expected_length;
static RobotCommandPayload latest_command;
static volatile uint32_t latest_command_ms;
static volatile bool command_available;
static volatile uint32_t crc_error_count;
static uint16_t tx_sequence;

/* ST-Link/CubeIDE snapshot of the most recent CRC-valid command. */
volatile RobotCommandPayload g_debug_latest_command;
volatile uint16_t g_debug_command_sequence;
volatile uint32_t g_debug_command_count;
volatile uint32_t g_debug_command_received_ms;
volatile uint32_t g_debug_crc_error_count;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ?
                (uint16_t)((crc << 1) ^ 0x1021U) :
                (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void reset_rx(void)
{
    rx_length = 0U;
    rx_expected_length = 0U;
}

static void process_complete_frame(uint32_t now_ms)
{
    ProtocolHeader header;
    uint16_t received_crc;
    uint16_t computed_crc;

    memcpy(&header, rx_frame, HEADER_SIZE);
    memcpy(&received_crc, &rx_frame[HEADER_SIZE + header.payload_length], CRC_SIZE);
    computed_crc = crc16_ccitt(
        &rx_frame[2],
        (uint16_t)(HEADER_SIZE - 2U + header.payload_length));

    if (received_crc != computed_crc) {
        ++crc_error_count;
        ++g_debug_crc_error_count;
        return;
    }

    if ((header.message_type == PROTOCOL_MSG_COMMAND) &&
        (header.payload_length == sizeof(RobotCommandPayload))) {
        RobotCommandPayload temporary;

        memcpy(&temporary, &rx_frame[HEADER_SIZE], sizeof(temporary));
        latest_command = temporary;
        latest_command_ms = now_ms;
        command_available = true;

        g_debug_latest_command = temporary;
        g_debug_command_sequence = header.sequence;
        g_debug_command_received_ms = now_ms;
        ++g_debug_command_count;
    }
}

void Protocol_Init(void)
{
    reset_rx();
    memset(&latest_command, 0, sizeof(latest_command));
    command_available = false;
    latest_command_ms = 0U;
    crc_error_count = 0U;
    tx_sequence = 0U;

    memset((void *)&g_debug_latest_command, 0, sizeof(g_debug_latest_command));
    g_debug_command_sequence = 0U;
    g_debug_command_count = 0U;
    g_debug_command_received_ms = 0U;
    g_debug_crc_error_count = 0U;
}

void Protocol_RxBytes(const uint8_t *data, uint16_t length, uint32_t now_ms)
{
    uint16_t input_index;

    for (input_index = 0U; input_index < length; ++input_index) {
        uint8_t byte = data[input_index];

        if ((rx_length == 0U) && (byte != MAGIC_BYTE_0)) {
            continue;
        }
        if ((rx_length == 1U) && (byte != MAGIC_BYTE_1)) {
            rx_length = (byte == MAGIC_BYTE_0) ? 1U : 0U;
            continue;
        }

        rx_frame[rx_length++] = byte;

        if (rx_length == HEADER_SIZE) {
            ProtocolHeader header;
            memcpy(&header, rx_frame, HEADER_SIZE);
            if ((header.magic != PROTOCOL_MAGIC) ||
                (header.version != PROTOCOL_VERSION) ||
                (header.payload_length > MAX_PAYLOAD_SIZE)) {
                reset_rx();
                continue;
            }
            rx_expected_length = (uint16_t)(
                HEADER_SIZE + header.payload_length + CRC_SIZE);
        }

        if ((rx_expected_length != 0U) && (rx_length == rx_expected_length)) {
            process_complete_frame(now_ms);
            reset_rx();
        } else if (rx_length >= MAX_FRAME_SIZE) {
            reset_rx();
        }
    }
}

bool Protocol_GetFreshCommand(
    uint32_t now_ms,
    uint32_t maximum_age_ms,
    RobotCommandPayload *output)
{
    bool valid;

    __disable_irq();
    valid = command_available &&
        ((uint32_t)(now_ms - latest_command_ms) <= maximum_age_ms);
    if (valid) {
        *output = latest_command;
    }
    __enable_irq();
    return valid;
}

bool Protocol_CommandIsFresh(uint32_t now_ms, uint32_t maximum_age_ms)
{
    bool valid;

    __disable_irq();
    valid = command_available &&
        ((uint32_t)(now_ms - latest_command_ms) <= maximum_age_ms);
    __enable_irq();
    return valid;
}

uint16_t Protocol_EncodeState(
    const RobotStatePayload *state,
    uint8_t *output,
    uint16_t capacity)
{
    const uint16_t frame_size = (uint16_t)(
        HEADER_SIZE + sizeof(RobotStatePayload) + CRC_SIZE);
    ProtocolHeader header;
    uint16_t crc;

    if ((state == NULL) || (output == NULL) || (capacity < frame_size)) {
        return 0U;
    }

    header.magic = PROTOCOL_MAGIC;
    header.version = PROTOCOL_VERSION;
    header.message_type = PROTOCOL_MSG_STATE;
    header.payload_length = (uint16_t)sizeof(RobotStatePayload);
    header.sequence = tx_sequence++;

    memcpy(output, &header, HEADER_SIZE);
    memcpy(&output[HEADER_SIZE], state, sizeof(RobotStatePayload));
    crc = crc16_ccitt(
        &output[2],
        (uint16_t)(HEADER_SIZE - 2U + sizeof(RobotStatePayload)));
    memcpy(&output[HEADER_SIZE + sizeof(RobotStatePayload)], &crc, CRC_SIZE);
    return frame_size;
}

uint32_t Protocol_GetCrcErrorCount(void)
{
    return crc_error_count;
}
