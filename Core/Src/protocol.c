#include "protocol.h"

uint8_t MotorIMU_Packet_t[60]; // 电机+IMU数据包

float MotorIMU_Packet_float[33]; // 电机+IMU数据包
//float数组0-23是12电机的数据，按顺序为：r_leg_pitch的位置 r_leg_pitch的速度, r_leg_roll的位置, r_leg_roll的速度......l_ankle_roll的位置,l_ankle_roll的速度
//float数组24-26是三轴加速度m/s², 27-29是三轴角速度rad/s, 30-33是四元数数据

/*
 * @brief  打包并通过USB发送 电机+IMU 数据
 * @param  MotorIMU_Packet_float 要发送的数据包
 * @retval None
 */
uint8_t USB_Send_MotorIMU_Packet_float(float *MotorIMU_Packet_float)
{
    MotorIMU_Packet_float[0] = -MotorIMU_Packet_float[0];
    MotorIMU_Packet_float[1] = -MotorIMU_Packet_float[1];
    MotorIMU_Packet_float[8] = -MotorIMU_Packet_float[8];
    MotorIMU_Packet_float[9] = -MotorIMU_Packet_float[9];
    MotorIMU_Packet_float[12] = -MotorIMU_Packet_float[12];
    MotorIMU_Packet_float[13] = -MotorIMU_Packet_float[13];
    MotorIMU_Packet_float[20] = -MotorIMU_Packet_float[20];
    MotorIMU_Packet_float[21] = -MotorIMU_Packet_float[21];// urdf有4个电机方向是反的，反转4个电机的角度和速度


    return 0;
}