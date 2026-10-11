#include "action_goto.h"

void ActionGoto_Begin(ActionGoto *motion, const float current[12],
                      const float target[12], uint8_t steps, uint32_t now_ms)
{
    for (uint8_t i = 0U; i < ACTION_GOTO_JOINTS; ++i) {
        motion->start[i] = current[i];
        motion->position[i] = current[i];
        motion->target[i] = target[i];
    }
    motion->start_ms = now_ms;
    motion->last_send_ms = now_ms;
    motion->steps = steps;
    /* A zero-duration request is rejected without jumping the motors. */
    motion->busy = steps != 0U;
}

uint8_t ActionGoto_Service(ActionGoto *motion, uint32_t now_ms)
{
    if (motion->busy == 0U ||
        (uint32_t)(now_ms - motion->last_send_ms) < ACTION_GOTO_STEP_MS) {
        return 0U;
    }
    uint32_t step = (uint32_t)(now_ms - motion->start_ms) / ACTION_GOTO_STEP_MS;
    if (step >= motion->steps) {
        for (uint8_t i = 0U; i < ACTION_GOTO_JOINTS; ++i) {
            motion->position[i] = motion->target[i];
        }
        motion->busy = 0U;
    } else {
        float fraction = (float)step / (float)motion->steps;
        for (uint8_t i = 0U; i < ACTION_GOTO_JOINTS; ++i) {
            motion->position[i] = motion->start[i] +
                (motion->target[i] - motion->start[i]) * fraction;
        }
    }
    motion->last_send_ms = now_ms;
    return 1U;
}

void ActionGoto_Cancel(ActionGoto *motion)
{
    motion->busy = 0U;
}
