#include <stdint.h>
#include "action_goto.h"

static ActionGoto motion;
static float current[12], target[12];

/* Fixtures set the inputs; all motion behavior comes from production C. */
uint32_t test_begin(uint32_t steps, uint32_t now_ms, uint32_t fractional)
{
    for (uint32_t i = 0; i < 12; ++i) {
        current[i] = fractional ? 0.1234567f * (float)(i + 1U) : 0.0f;
        target[i] = fractional ? -0.4567891f * (float)(i + 1U) : (float)(i + 1U);
    }
    ActionGoto_Begin(&motion, current, target, (uint8_t)steps, now_ms);
    return motion.busy;
}
uint32_t test_service(uint32_t now_ms) { return ActionGoto_Service(&motion, now_ms); }
uint32_t test_busy(void) { return motion.busy; }
uint32_t test_cancel(void) { ActionGoto_Cancel(&motion); return motion.busy; }
uint32_t test_restart(uint32_t steps, uint32_t now_ms)
{
    ActionGoto_Begin(&motion, motion.position, current, (uint8_t)steps, now_ms);
    return motion.busy;
}
uint32_t test_position(uint32_t index)
{
    union { float number; uint32_t bits; } value;
    value.number = motion.position[index];
    return value.bits;
}
uint32_t test_target(uint32_t index)
{
    union { float number; uint32_t bits; } value;
    value.number = target[index];
    return value.bits;
}
uint32_t test_start(uint32_t index)
{
    union { float number; uint32_t bits; } value;
    value.number = current[index];
    return value.bits;
}
