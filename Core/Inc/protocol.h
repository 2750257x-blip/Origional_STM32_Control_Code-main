#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <usbd_cdc_if.h>

//==================== 协议帧定义 ====================


//==================== 数据结构体 ====================

 extern uint8_t MotorIMU_Packet_t[60]; // 电机+IMU数据包
 extern float MotorIMU_Packet_float[34]; // 电机+IMU数据包

//==================== 函数接口 ====================
// USB发送 电机+IMU 数据包
uint8_t USB_Send_MotorIMU_Packet_float(float *MotorIMU_Packet_float);

#endif