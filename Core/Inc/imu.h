#ifndef __IMU_H
#define __IMU_H

#include "stm32h7xx_hal.h"

#define ACCEL_CAN_MAX   (235.2f)        // 加速度量程 ±24g，1g = 9.8m/s²
#define ACCEL_CAN_MIN	(-235.2f)
#define GYRO_CAN_MAX	(34.88f)        // 陀螺仪量程 ±2000°/s，1°/s = 0.01745 rad/s
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)         // 俯仰角范围 ±90°，1° = 0.01745 rad
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)        // 横滚角范围 ±180°
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)        // 偏航角范围 ±180°
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN		(0.0f)          // 温度范围 0~60℃
#define TEMP_MAX		(60.0f)
#define Quaternion_MIN	(-1.0f)         // 四元数范围 -1 ~ 1
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0
#define CMD_WRITE 1
extern volatile int32_t imu_data_count;

#define IMU_DATA_ACCEL_READY       (1U << 0)
#define IMU_DATA_GYRO_READY        (1U << 1)
#define IMU_DATA_QUATERNION_READY  (1U << 2)
#define IMU_DATA_REQUIRED          (IMU_DATA_ACCEL_READY | IMU_DATA_GYRO_READY | IMU_DATA_QUATERNION_READY)
typedef enum
{
	COM_USB=0,    // USB端口
	COM_RS485,    // RS485端口
	COM_CAN,      // CAN端口
	COM_VOFA      // VOFA上位机

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,    // CAN波特率 1Mbps
	CAN_BAUD_500K,    // CAN波特率 500Kbps
	CAN_BAUD_400K,    // CAN波特率 400Kbps
	CAN_BAUD_250K,    // CAN波特率 250Kbps
	CAN_BAUD_200K,    // CAN波特率 200Kbps
	CAN_BAUD_100K,    // CAN波特率 100Kbps
	CAN_BAUD_50K,     // CAN波特率 50Kbps
	CAN_BAUD_25K      // CAN波特率 25Kbps

}imu_baudrate_e;

typedef enum
{
	REBOOT_IMU=0,              // 重启IMU                  写1
	ACCEL_DATA,                // 请求加速度数据           读0
	GYRO_DATA,                 // 请求陀螺仪数据           读0
	EULER_DATA,                // 请求欧拉角数据           读0
	QUAT_DATA,                 // 请求四元数数据           读0

	SET_ZERO,                  // 设置零位                 写1
	ACCEL_CALI,                // 加速度计校准             写1
	GYRO_CALI,                 // 陀螺仪校准               写1
	MAG_CALI,                  // 磁力计校准               写1

	CHANGE_COM,                // 切换通信端口             写01
	SET_DELAY,                 // 设置主动模式回传延时     写01
	CHANGE_ACTIVE,             // 主动/请求模式切换        写01

	SET_BAUD,                  // 设置CAN波特率            写01
	SET_CAN_ID,                // 设置CAN ID               写01
	SET_MST_ID,                // 设置主机ID               写01
	DATA_OUTPUT_SELECTION,     // 选择输出数据类型(欧拉/四元数) 写01
	SAVE_PARAM=254,            // 保存参数到Flash         写1
	RESTORE_SETTING=255        // 恢复出厂设置             写1
}reg_id_e;



typedef struct
{
	uint8_t can_id;
	uint8_t mst_id;

	FDCAN_HandleTypeDef *can_handle;

	float pitch;
	float roll;
	float yaw;

	float gyro[3];
	float accel[3];

	float q[4];              // 四元数数据(w,x,y,z)

	float cur_temp;          // 当前芯片温度

}imu_t;

extern imu_t imu;
extern volatile uint8_t imu_data_ready;
extern volatile uint32_t imu_last_accel_ms;
extern volatile uint32_t imu_last_gyro_ms;
extern volatile uint32_t imu_last_quaternion_ms;
extern uint8_t imu_buf[12];

void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan);  // IMU初始化
void imu_write_reg(uint8_t reg_id,uint32_t data);                          // IMU写寄存器
void imu_read_reg(uint8_t reg_id);                                         // IMU读寄存器

void imu_reboot();                                                         // 重启IMU
void imu_set_zero();                                                       // 设置零位
void imu_accel_calibration();                                              // 加速度计校准
void imu_gyro_calibration();                                               // 陀螺仪校准

void imu_change_com_port(imu_com_port_e port);                             // 切换通信端口
void imu_set_active_mode_delay(uint32_t delay);                            // 设置主动模式回传延时
void imu_change_to_active();                                               // 切换为主动上报模式
void imu_change_to_request();                                              // 切换为请求(问答)模式

void imu_set_baud(imu_baudrate_e baud);                                    // 设置CAN波特率
void imu_set_can_id(uint8_t can_id);                                       // 设置CAN ID
void imu_set_mst_id(uint8_t mst_id);                                       // 设置主机ID
void imu_save_parameters();                                                // 保存参数到Flash
void imu_restore_settings();                                               // 恢复出厂设置

void imu_request_accel();                                                 // 请求加速度数据
void imu_request_gyro();                                                  // 请求陀螺仪数据
void imu_request_euler();                                                  // 请求欧拉角数据
void imu_request_quat();                                                  // 请求四元数数据
void IMU_UpdateData(uint8_t* pData);									   // 解析IMU数据

uint8_t IMU_DataIsFresh(uint32_t now_ms, uint32_t maximum_age_ms);

#endif
