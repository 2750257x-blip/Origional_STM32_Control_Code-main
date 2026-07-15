#ifndef __QR_CODE_STATE_H
#define __QR_CODE_STATE_H

typedef enum {
    ACTION_IDLE = 0,
    ACTION_RAISE_LEFT_HAND,
    ACTION_RAISE_RIGHT_HAND,
    ACTION_LIFT_LEFT_LEG,
    ACTION_LIFT_RIGHT_LEG,
    ACTION_RAISE_BOTH_HANDS,
    ACTION_SHAKE_HEAD
} Action_StateTypeDef;

extern Action_StateTypeDef action_state;

void QRCode_State_Machine(void);

void Action_RaiseLeftHand(void);
void Action_RaiseRightHand(void);
void Action_LiftLeftLeg(void);
void Action_LiftRightLeg(void);
void Action_RaiseBothHands(void);
void Action_ShakeHead(void);

#endif /* __QR_CODE_STATE_H */