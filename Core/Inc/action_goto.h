#ifndef ACTION_GOTO_H
#define ACTION_GOTO_H

#include <stdint.h>

#define ACTION_GOTO_JOINTS 12U
#define ACTION_GOTO_STEP_MS 20U

/* Motor coordinates: right six joints followed by left six joints. */
typedef struct {
    float start[ACTION_GOTO_JOINTS];
    float target[ACTION_GOTO_JOINTS];
    float position[ACTION_GOTO_JOINTS];
    uint32_t start_ms;
    uint32_t last_send_ms;
    uint8_t steps;
    uint8_t busy;
} ActionGoto;

void ActionGoto_Begin(ActionGoto *motion, const float current[12],
                      const float target[12], uint8_t steps, uint32_t now_ms);
/* Advance at most one sample per call. Missed ticks never create a wait loop. */
uint8_t ActionGoto_Service(ActionGoto *motion, uint32_t now_ms);
void ActionGoto_Cancel(ActionGoto *motion);

#endif
