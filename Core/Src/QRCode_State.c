#include "QRCode_State.h"
#include "motor_el05.h"

Action_StateTypeDef action_state = ACTION_IDLE;

void QRCode_State_Machine(void)
{
    switch(action_state)
    {
        case ACTION_RAISE_LEFT_HAND:
            Action_RaiseLeftHand();
            break;
        case ACTION_RAISE_RIGHT_HAND:
            Action_RaiseRightHand();
            break;
        case ACTION_LIFT_LEFT_LEG:
            Action_LiftLeftLeg();
            break;
        case ACTION_LIFT_RIGHT_LEG:
            Action_LiftRightLeg();
            break;
        case ACTION_RAISE_BOTH_HANDS:
            Action_RaiseBothHands();
            break;
        case ACTION_SHAKE_HEAD:
            Action_ShakeHead();
            break;
        default:
            // 无效二维码ID，不执行动作
            break;
    }
}

/*************************************************
 * 1号二维码动作：举起左手，维持3秒以�??????????
 *************************************************/
void Action_RaiseLeftHand(void)
{
    //下面写举左手代码
    
}

/*************************************************
 * 2号二维码动作：举起右手，维持3秒以�??????????
 *************************************************/
void Action_RaiseRightHand(void)
{
   //下面写举右手代码

}

/*************************************************
 * 3号二维码动作：抬起左腿，维持3秒以�??????????
 *************************************************/
void Action_LiftLeftLeg(void)
{
    //下面写举左腿代码

}

/*************************************************
 * 4号二维码动作：抬起右腿，维持3秒以�??????????
 *************************************************/
void Action_LiftRightLeg(void)
{
    //下面写举右腿代码

}

/*************************************************
 * 5号二维码动作：举起双手，维持3秒以�??????????
 *************************************************/
void Action_RaiseBothHands(void)
{
    //下面写举双手代码

}

/*************************************************
 * 6号二维码动作：左右摇头，维持3秒以�??????????
 *************************************************/
void Action_ShakeHead(void)
{
    //下面写摇头代�?

}
