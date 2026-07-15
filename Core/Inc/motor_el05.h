#ifndef __MOTOR_EL05_H
#define __MOTOR_EL05_H
#include "stdint.h"
#include "fdcan.h"

// 电机参数定义（与手册一致）
#define P_MIN        -12.56f    //position范围（-4π rad）
#define P_MAX        12.56f     //position范围（π rad）
#define V_MIN        -50.0f     //velocity范围（-50 rad/s）
#define V_MAX        50.0f      //velocity范围（50 rad/s）

#define KP_MIN       0.0f       //位置增益范围
#define KP_MAX       500.0f     //位置增益范围
#define KD_MIN       0.0f       //速度增益范围
#define KD_MAX       5.0f       //速度增益范围
#define T_MIN        -6.0f      //力矩范围
#define T_MAX        6.0f       //力矩范围

// 各关节安全限位（根据实际机械结构设定）
// leg_pitch: 大腿前后摆动
#define SAFE_LEG_PITCH_MIN   -1.6f
#define SAFE_LEG_PITCH_MAX    1.6f
// leg_roll: 大腿侧摆
#define SAFE_LEG_ROLL_MIN    -0.5f
#define SAFE_LEG_ROLL_MAX     0.5f
// leg_yaw: 大腿旋转
#define SAFE_LEG_YAW_MIN     -1.0f
#define SAFE_LEG_YAW_MAX      1.0f
// knee_pitch: 膝盖弯曲（通常只能向后弯）
#define SAFE_KNEE_MIN         -1.6f
#define SAFE_KNEE_MAX         1.6f
// ankle_pitch: 脚踝前后
#define SAFE_ANKLE_PITCH_MIN -1.0f
#define SAFE_ANKLE_PITCH_MAX  1.0f
// ankle_roll: 脚踝侧向
#define SAFE_ANKLE_ROLL_MIN  -0.5f
#define SAFE_ANKLE_ROLL_MAX   0.5f

#define MASTER_ID    0xFD    // 主机CAN ID，固定0xFD

#define r_leg_pitch    0x01    // 电机ID定义
#define r_leg_roll     0x02
#define r_leg_yaw      0x03
#define r_knee_pitch   0X04
#define r_ankle_pitch   0x05
#define r_ankle_roll    0x06

#define l_leg_pitch    0x11
#define l_leg_roll     0x12
#define l_leg_yaw      0x13
#define l_knee_pitch   0X14
#define l_ankle_pitch   0x15
#define l_ankle_roll    0x16

// 电机通信类型（手册4.1）
#define CMD_CTRL     0x01    // 运控模式控制
#define CMD_STATUS   0x02    // 电机状态反馈（电机上报）
#define CMD_ENABLE   0x03    // 电机使能
#define CMD_STOP     0x04    // 电机停止
#define CMD_SET_ZERO 0x06    // 设置机械零位
#define CMD_TX       0x18    // 设置主动上报

// 电机状态结构体
typedef struct
{
    float pos;    // 位置(rad)
    float vel;    // 速度(rad/s)
    float torque; // 力矩(N.m)
    uint16_t temp;// 温度(℃)
    uint8_t fault;// 故障标志
}EL05_Motor_Status;

extern volatile uint16_t motor_status_ready;
extern volatile uint16_t motor_status_fault;
extern volatile uint16_t motor_status_mode;
extern volatile uint16_t motor_fault_test;
extern uint8_t motor_status_buf[48];

// 函数声明
void EL05_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);      // 电机使能
void EL05_Motor_Stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);        // 电机停止
void EL05_Motor_Clear_Fault(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id); // 电机清故障
void EL05_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);     // 设置零位
void EL05_Motor_TX(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);          // 设置主动上报
// 运控模式控制（核心）
void EL05_Motor_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float torque, float pos, float vel, float kp, float kd);
void Motor_Ctrl( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t *control);
void Motor_limitCtrl( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,  float kp,  float kd, uint8_t *control);
// 解析电机反馈帧（通信类型2）
uint8_t EL05_Parse_Status(uint32_t can_id, uint8_t *data, EL05_Motor_Status *status);

void motor_setzero();
void motor_enable();
void motor_control(uint8_t *control_buf);
void motor_limitcontrol(uint8_t *control_buf);
void Motor_limitCtrl_float( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float pos, float vel, float kp, float kd);
void motor_limitcontrol_float(float *control_buf_float);
#endif /* __MOTOR_EL05_H */