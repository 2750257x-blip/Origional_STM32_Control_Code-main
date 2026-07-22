#include "motor_el05.h"
#include "math.h"

volatile uint16_t motor_status_ready = 0;
volatile uint16_t motor_status_fault = 0;
volatile uint16_t motor_status_mode = 0;
volatile uint16_t motor_fault_test = 0;
uint8_t motor_status_buf[48] = {0}; 

// 浮点数转无符号整数（手册4.4，16位）
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// 无符号整数转浮点数
static float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (float)x * span / ((float)((1 << bits) - 1)) + offset;
}

// 根据电机ID获取安全限位
static void get_safe_limit(uint8_t motor_id, float *p_min, float *p_max)
{
    switch(motor_id)
    {
        case r_leg_pitch:
        case l_leg_pitch:
            *p_min = SAFE_LEG_PITCH_MIN;
            *p_max = SAFE_LEG_PITCH_MAX;
            break;
        case r_leg_roll:
        case l_leg_roll:
            *p_min = SAFE_LEG_ROLL_MIN;
            *p_max = SAFE_LEG_ROLL_MAX;
            break;
        case r_leg_yaw:
        case l_leg_yaw:
            *p_min = SAFE_LEG_YAW_MIN;
            *p_max = SAFE_LEG_YAW_MAX;
            break;
        case r_knee_pitch:
        case l_knee_pitch:
            *p_min = SAFE_KNEE_MIN;
            *p_max = SAFE_KNEE_MAX;
            break;
        case r_ankle_pitch:
        case l_ankle_pitch:
            *p_min = SAFE_ANKLE_PITCH_MIN;
            *p_max = SAFE_ANKLE_PITCH_MAX;
            break;
        case r_ankle_roll:
        case l_ankle_roll:
            *p_min = SAFE_ANKLE_ROLL_MIN;
            *p_max = SAFE_ANKLE_ROLL_MAX;
            break;
        default:
            *p_min = P_MIN;
            *p_max = P_MAX;
            break;
    }
}

// 拼接29位CAN扩展ID（手册4.1：通信类型(5bit)+主机ID(8bit)+电机ID(8bit)，其余保留）
static uint32_t EL05_Combine_CANID(uint8_t cmd_type, uint8_t master_id, uint8_t motor_id)
{
  uint32_t ext_id = 0;
  ext_id |= ((uint32_t)cmd_type & 0x1F) << 24;  // bit28~24：通信类型
  ext_id |= ((uint32_t)master_id & 0xFF) << 8; // bit15~8：主机ID
  ext_id |= (uint32_t)motor_id & 0xFF;   // bit7~0：电机ID
  return ext_id;
}

static uint32_t EL05_Combine_CANID_Ctrl(uint8_t cmd_type, uint16_t u_tor, uint8_t motor_id)
{
  uint32_t ext_id = 0;
  ext_id |= ((uint32_t)cmd_type & 0x1F) << 24;  // bit28~24：通信类型
  ext_id |= ((uint32_t)u_tor & 0xFFFF) << 8; // bit23~8：前馈力矩（16位）
  ext_id |= (uint32_t)motor_id & 0xFF;   // bit7~0：电机ID
  return ext_id;
}

// 运控模式控制（通信类型1，核心指令）
// torque：前馈力矩，pos：目标位置，vel：目标速度，kp：位置增益，kd：速度增益
void EL05_Motor_Ctrl( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float torque, float pos, float vel, float kp, float kd)
{
  // 位置限位保护
  float safe_min, safe_max;
  get_safe_limit(motor_id, &safe_min, &safe_max);
  if(pos < safe_min) pos = safe_min;
  if(pos > safe_max) pos = safe_max;

  uint8_t data[8] = {0};
  // 按手册格式拼接数据（高字节在前）
  uint16_t u_pos = float_to_uint(pos, P_MIN, P_MAX, 16);
  uint16_t u_vel = float_to_uint(vel, V_MIN, V_MAX, 16);
  uint16_t u_kp  = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  uint16_t u_kd  = float_to_uint(kd, KD_MIN, KD_MAX, 16);
  uint16_t u_tor = float_to_uint(torque, T_MIN, T_MAX, 16);
  uint32_t can_id = EL05_Combine_CANID_Ctrl(CMD_CTRL, u_tor, motor_id);
  data[0] = (u_pos >> 8) & 0xFF;
  data[1] = u_pos & 0xFF;
  data[2] = (u_vel >> 8) & 0xFF;
  data[3] = u_vel & 0xFF;
  data[4] = (u_kp >> 8) & 0xFF;
  data[5] = u_kp & 0xFF;
  data[6] = (u_kd >> 8) & 0xFF;
  data[7] = u_kd & 0xFF;
  // 发送控制帧
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

void Motor_Ctrl( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, uint8_t *control)
{
  uint8_t data[8] = {0};
  uint16_t u_tor = (uint16_t)(control[0] << 8 | control[1]);
  uint16_t u_pos = (uint16_t)(control[2] << 8 | control[3]);

  // 位置限位保护：解码→限幅→重新编码
  float pos = uint_to_float(u_pos, P_MIN, P_MAX, 16);
  float safe_min, safe_max;
  get_safe_limit(motor_id, &safe_min, &safe_max);
  if(pos < safe_min) pos = safe_min;
  if(pos > safe_max) pos = safe_max;
  u_pos = float_to_uint(pos, P_MIN, P_MAX, 16);

  uint32_t can_id = EL05_Combine_CANID_Ctrl(CMD_CTRL, u_tor, motor_id);
  data[0] = (u_pos >> 8) & 0xFF;
  data[1] = u_pos & 0xFF;
  data[2] = control[4];
  data[3] = control[5];
  data[4] = control[6];
  data[5] = control[7];
  data[6] = control[8];
  data[7] = control[9];
  // 发送控制帧
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}
void Motor_limitCtrl( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,  float kp,  float kd, uint8_t *control)
{
  uint8_t data[8] = {0};
  uint16_t u_pos = (uint16_t)(control[0] << 8 | control[1]);
  uint16_t u_kp  = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  uint16_t u_kd  = float_to_uint(kd, KD_MIN, KD_MAX, 16);

  // 位置限位保护：解码→限幅→重新编码
  float pos = uint_to_float(u_pos, P_MIN, P_MAX, 16);
  float safe_min, safe_max;
  get_safe_limit(motor_id, &safe_min, &safe_max);
  if(pos < safe_min) pos = safe_min;
  if(pos > safe_max) pos = safe_max;
  u_pos = float_to_uint(pos, P_MIN, P_MAX, 16);

  uint32_t can_id = EL05_Combine_CANID_Ctrl(CMD_CTRL, 0X7FFF, motor_id);
  data[0] = (u_pos >> 8) & 0xFF;
  data[1] = u_pos & 0xFF;
  data[2] = control[2];
  data[3] = control[3];
  data[4] = (u_kp >> 8) & 0xFF;
  data[5] = u_kp & 0xFF;
  data[6] = (u_kd >> 8) & 0xFF;
  data[7] = u_kd & 0xFF;
  // 发送控制帧
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

  
  void Motor_limitCtrl_float( FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float pos, float vel, float kp, float kd)
 {
  // 位置限位保护
  float safe_min, safe_max;
  get_safe_limit(motor_id, &safe_min, &safe_max);
  if(pos < safe_min) pos = safe_min;
  if(pos > safe_max) pos = safe_max;

  uint8_t data[8] = {0};
  // 按手册格式拼接数据（高字节在前）
  uint16_t u_pos = float_to_uint(pos, P_MIN, P_MAX, 16);
  uint16_t u_vel = float_to_uint(vel, V_MIN, V_MAX, 16);
  uint16_t u_kp  = float_to_uint(kp, KP_MIN, KP_MAX, 16);
  uint16_t u_kd  = float_to_uint(kd, KD_MIN, KD_MAX, 16);
  uint16_t u_tor = float_to_uint(0, T_MIN, T_MAX, 16);
  uint32_t can_id = EL05_Combine_CANID_Ctrl(CMD_CTRL, u_tor, motor_id);
  data[0] = (u_pos >> 8) & 0xFF;
  data[1] = u_pos & 0xFF;
  data[2] = (u_vel >> 8) & 0xFF;
  data[3] = u_vel & 0xFF;
  data[4] = (u_kp >> 8) & 0xFF;
  data[5] = u_kp & 0xFF;
  data[6] = (u_kd >> 8) & 0xFF;
  data[7] = u_kd & 0xFF;
  // 发送控制帧
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

// 解析电机反馈帧（通信类型2，手册4.1）
uint8_t EL05_Parse_Status(uint32_t can_id, uint8_t *data, EL05_Motor_Status *status)
{
  // 校验通信类型为2（bit28~24=0x02）
  uint8_t cmd_type = (can_id >> 24) & 0x1F;
  if(cmd_type != CMD_STATUS || status == NULL) return 1;
  
  // 解析位置(rad)：data[0-1]，16位无符号转浮点数
  uint16_t u_pos = (data[0] << 8) | data[1];
  status->pos = (float)u_pos * (P_MAX - P_MIN) / 65535.0f + P_MIN;
  // 解析速度(rad/s)：data[2-3]
  uint16_t u_vel = (data[2] << 8) | data[3];
  status->vel = (float)u_vel * (V_MAX - V_MIN) / 65535.0f + V_MIN;
  // 解析力矩(N.m)：data[4-5]
  uint16_t u_tor = (data[4] << 8) | data[5];
  status->torque = (float)u_tor * (T_MAX - T_MIN) / 65535.0f + T_MIN;
  // 解析温度(℃)：data[6-7]，手册定义：Temp*10
  uint16_t u_temp = (data[6] << 8) | data[7];
  status->temp = u_temp / 10;
  // 解析故障标志（bit21~16在can_id的bit23~18）
  status->fault = (can_id >> 16) & 0x3F;
  return 0;
}

// 电机使能（通信类型3）
void EL05_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {0};
  uint32_t can_id = EL05_Combine_CANID(CMD_ENABLE, MASTER_ID, motor_id);
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

// 电机停止（通信类型4）
void EL05_Motor_Stop(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {0};
  uint32_t can_id = EL05_Combine_CANID(CMD_STOP, MASTER_ID, motor_id);
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}
// 电机清故障
void EL05_Motor_Clear_Fault(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {0};
  data[0] = 0X01;
  uint32_t can_id = EL05_Combine_CANID(CMD_STOP, MASTER_ID, motor_id);
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

// 设置机械零位（通信类型6，data[0]=1）
void EL05_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {1,0,0,0,0,0,0,0};
  uint32_t can_id = EL05_Combine_CANID(CMD_SET_ZERO, MASTER_ID, motor_id);
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

// 设置主动上报（通信类型24，data[0]=1）
void EL05_Motor_TX(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {1,2,3,4,5,6,0,1};
  uint32_t can_id = EL05_Combine_CANID(CMD_TX, MASTER_ID, motor_id);
  FDCAN_Send_ExtFrame(hfdcan, can_id, data, 8);
}

void motor_setzero()
{
  EL05_Motor_SetZero(&hfdcan1, r_leg_pitch);
  // EL05_Motor_SetZero(&hfdcan1, r_leg_roll);
  // EL05_Motor_SetZero(&hfdcan1, r_leg_yaw);
  // EL05_Motor_SetZero(&hfdcan1, r_knee_pitch);
  // EL05_Motor_SetZero(&hfdcan1, r_ankle_pitch);
  // EL05_Motor_SetZero(&hfdcan1, r_ankle_roll);

  EL05_Motor_SetZero(&hfdcan2, l_leg_pitch );
  // EL05_Motor_SetZero(&hfdcan2, l_leg_roll);
  // EL05_Motor_SetZero(&hfdcan2, l_leg_yaw);
  // EL05_Motor_SetZero(&hfdcan2, l_knee_pitch);
  // EL05_Motor_SetZero(&hfdcan2, l_ankle_pitch);
  // EL05_Motor_SetZero(&hfdcan2, l_ankle_roll);
}

void motor_enable()
{
  EL05_Motor_Enable(&hfdcan1, r_leg_pitch);
  EL05_Motor_Enable(&hfdcan1, r_leg_roll);
  EL05_Motor_Enable(&hfdcan1, r_leg_yaw);
  EL05_Motor_Enable(&hfdcan1, r_knee_pitch);
  EL05_Motor_Enable(&hfdcan1, r_ankle_pitch);
  EL05_Motor_Enable(&hfdcan1, r_ankle_roll);

  EL05_Motor_Enable(&hfdcan2, l_leg_pitch );
  EL05_Motor_Enable(&hfdcan2, l_leg_roll);
  EL05_Motor_Enable(&hfdcan2, l_leg_yaw);
  EL05_Motor_Enable(&hfdcan2, l_knee_pitch);
  EL05_Motor_Enable(&hfdcan2, l_ankle_pitch);
  EL05_Motor_Enable(&hfdcan2, l_ankle_roll);
}

void motor_control(uint8_t *control_buf) {
  // 解析 control_buf 中的控制命令，并控制电机
  Motor_Ctrl(&hfdcan1, r_leg_pitch, control_buf);
  Motor_Ctrl(&hfdcan1, r_leg_roll, control_buf + 10);
  Motor_Ctrl(&hfdcan1, r_leg_yaw, control_buf + 20);
  Motor_Ctrl(&hfdcan1, r_knee_pitch, control_buf + 30);
  Motor_Ctrl(&hfdcan1, r_ankle_pitch, control_buf + 40);
  Motor_Ctrl(&hfdcan1, r_ankle_roll, control_buf + 50);

  Motor_Ctrl(&hfdcan2, l_leg_pitch, control_buf + 60);
  Motor_Ctrl(&hfdcan2, l_leg_roll, control_buf + 70);
  Motor_Ctrl(&hfdcan2, l_leg_yaw, control_buf + 80);
  Motor_Ctrl(&hfdcan2, l_knee_pitch, control_buf + 90);
  Motor_Ctrl(&hfdcan2, l_ankle_pitch, control_buf + 100);
  Motor_Ctrl(&hfdcan2, l_ankle_roll, control_buf + 110);
}
void motor_limitcontrol(uint8_t *control_buf) {
  // 解析 control_buf 中的控制命令，并控制电机
  control_buf[0] = ~control_buf[0];
  control_buf[1] = ~control_buf[1];
  control_buf[2] = ~control_buf[2];
  control_buf[3] = ~control_buf[3];
  Motor_limitCtrl(&hfdcan1, r_leg_pitch, 35.0f, 1.5f, control_buf);
  Motor_limitCtrl(&hfdcan1, r_leg_roll, 30.0f, 1.2f, control_buf + 4);
  Motor_limitCtrl(&hfdcan1, r_leg_yaw, 20.0f, 1.0f, control_buf + 8);
  Motor_limitCtrl(&hfdcan1, r_knee_pitch, 35.0f, 1.5f, control_buf + 12);
  control_buf[16] = ~control_buf[16];
  control_buf[17] = ~control_buf[17];
  control_buf[18] = ~control_buf[18];
  control_buf[19] = ~control_buf[19];
  Motor_limitCtrl(&hfdcan1, r_ankle_pitch, 15.0f, 0.8f, control_buf + 16);
  Motor_limitCtrl(&hfdcan1, r_ankle_roll, 12.0f, 0.7f, control_buf + 20);

  control_buf[24] = ~control_buf[24];
  control_buf[25] = ~control_buf[25];
  control_buf[26] = ~control_buf[26];
  control_buf[27] = ~control_buf[27];
  Motor_limitCtrl(&hfdcan2, l_leg_pitch, 35.0f, 1.5f, control_buf + 24);
  Motor_limitCtrl(&hfdcan2, l_leg_roll, 30.0f, 1.2f, control_buf + 28);
  Motor_limitCtrl(&hfdcan2, l_leg_yaw, 20.0f, 1.0f, control_buf + 32);
  Motor_limitCtrl(&hfdcan2, l_knee_pitch, 35.0f, 1.5f, control_buf + 36);
  control_buf[40] = ~control_buf[40];
  control_buf[41] = ~control_buf[41];
  control_buf[42] = ~control_buf[42];
  control_buf[43] = ~control_buf[43];
  Motor_limitCtrl(&hfdcan2, l_ankle_pitch, 15.0f, 0.8f, control_buf + 40);
  Motor_limitCtrl(&hfdcan2, l_ankle_roll, 12.0f, 0.7f, control_buf + 44);
}

void motor_limitcontrol_float(float *control_buf_float) 
{
  control_buf_float[12] = -control_buf_float[12];
  control_buf_float[13] = -control_buf_float[13];
  Motor_limitCtrl_float(&hfdcan1, r_leg_pitch, control_buf_float[12], control_buf_float[13], 35.0f, 1.5f);
  Motor_limitCtrl_float(&hfdcan1, r_leg_roll, control_buf_float[14], control_buf_float[15],30.0f, 1.2f);
  Motor_limitCtrl_float(&hfdcan1, r_leg_yaw, control_buf_float[16], control_buf_float[17], 20.0f, 1.0f);
  Motor_limitCtrl_float(&hfdcan1, r_knee_pitch, control_buf_float[18], control_buf_float[19],35.0f, 1.5f);
  control_buf_float[20] = -control_buf_float[20];
  control_buf_float[21] = -control_buf_float[21];
  Motor_limitCtrl_float(&hfdcan1, r_ankle_pitch, control_buf_float[20], control_buf_float[21],15.0f, 0.8f);
  Motor_limitCtrl_float(&hfdcan1, r_ankle_roll, control_buf_float[22], control_buf_float[23], 12.0f, 0.7f);

  control_buf_float[0] = -control_buf_float[0];
  control_buf_float[1] = -control_buf_float[1];
  Motor_limitCtrl_float(&hfdcan2, l_leg_pitch, control_buf_float[0], control_buf_float[1], 35.0f, 1.5f);
  Motor_limitCtrl_float(&hfdcan2, l_leg_roll, control_buf_float[2], control_buf_float[3], 30.0f, 1.2f);
  Motor_limitCtrl_float(&hfdcan2, l_leg_yaw, control_buf_float[4], control_buf_float[5], 20.0f, 1.0f);
  Motor_limitCtrl_float(&hfdcan2, l_knee_pitch, control_buf_float[6], control_buf_float[7], 35.0f, 1.5f);
  control_buf_float[8] = -control_buf_float[8];
  control_buf_float[9] = -control_buf_float[9];
  Motor_limitCtrl_float(&hfdcan2, l_ankle_pitch, control_buf_float[8], control_buf_float[9], 15.0f, 0.8f);
  Motor_limitCtrl_float(&hfdcan2, l_ankle_roll, control_buf_float[10], control_buf_float[11], 12.0f, 0.7f);
}

