#ifndef __IMU_H
#define __IMU_H

#include "stm32h7xx_hal.h"

#define ACCEL_CAN_MAX   (235.2f)        //加速度范围（±24g，1g=9.8m/s²）
#define ACCEL_CAN_MIN	(-235.2f)       
#define GYRO_CAN_MAX	(34.88f)        //角速度范围（±2000°/s，1°/s=0.01745 rad/s）
#define GYRO_CAN_MIN	(-34.88f) 
#define PITCH_CAN_MAX	(90.0f)         //姿态范围（±90°，1°=0.01745 rad）
#define PITCH_CAN_MIN	(-90.0f)      
#define ROLL_CAN_MAX	(180.0f)        //姿态范围（±180°，1°=0.01745 rad）
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)        //姿态范围（±180°，1°=0.01745 rad）
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN		(0.0f)          //温度范围（0-60℃）  
#define TEMP_MAX		(60.0f)
#define Quaternion_MIN	(-1.0f)         //四元数范围（-1到1）
#define Quaternion_MAX	(1.0f)

#define CMD_READ 0                    
#define CMD_WRITE 1

typedef enum
{
	COM_USB=0,    // USB虚拟串口
	COM_RS485,    // RS485串口
	COM_CAN,      // CAN总线
	COM_VOFA      // VOFA无线通信

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,    // CAN波特率1Mbps
	CAN_BAUD_500K,    // CAN波特率500Kbps
	CAN_BAUD_400K,    // CAN波特率400Kbps
	CAN_BAUD_250K,    // CAN波特率250Kbps
	CAN_BAUD_200K,    // CAN波特率200Kbps
	CAN_BAUD_100K,    // CAN波特率100Kbps
	CAN_BAUD_50K,     // CAN波特率50Kbps
	CAN_BAUD_25K      // CAN波特率25Kbps

}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,              //重启IMU                  1
	ACCEL_DATA,                //请求加速度数据            0
	GYRO_DATA,                 //请求角速度数据            0
	EULER_DATA,                //请求欧拉角数据            0
	QUAT_DATA,                 //请求四元数数据            0

	SET_ZERO,                  //角度设置零位              1
	ACCEL_CALI,                //加速度校准                1
	GYRO_CALI,                 //陀螺仪校准                1
	MAG_CALI,                  //磁力计校准                1

	CHANGE_COM,                //切换通信端口              01
	SET_DELAY,                 //设置主动模式下数据输出延时 01
	CHANGE_ACTIVE,             //切换主动/请求模式         01

	SET_BAUD,                  //设置CAN波特率             01
	SET_CAN_ID,                //设置CAN ID                01
	SET_MST_ID,                //设置主机ID                 01
	DATA_OUTPUT_SELECTION,     //数据输出选择（欧拉角/四元数） 01
	SAVE_PARAM=254,            //保存参数到非易失存储器         1
	RESTORE_SETTING=255        //恢复出厂设置                  	1
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

	float q[4];              //四元数数据（单位：无）

	float cur_temp;          //当前温度（单位：℃）

}imu_t;

extern imu_t imu;
extern volatile uint8_t imu_data_ready;
extern uint8_t imu_buf[12];

void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan);  // IMU初始化
void imu_write_reg(uint8_t reg_id,uint32_t data);                          // IMU寄存器写入
void imu_read_reg(uint8_t reg_id);                                         // IMU寄存器读取

void imu_reboot();                                                         // 重启IMU
void imu_set_zero();                                                       // 设置零位
void imu_accel_calibration();                                              // 加速度校准
void imu_gyro_calibration();                                               // 陀螺仪校准

void imu_change_com_port(imu_com_port_e port);                             // 切换通信端口
void imu_set_active_mode_delay(uint32_t delay);                            // 设置主动模式下数据输出延时
void imu_change_to_active();                                               // 切换到主动模式
void imu_change_to_request();                                              // 切换到请求模式

void imu_set_baud(imu_baudrate_e baud);                                    // 设置CAN波特率
void imu_set_can_id(uint8_t can_id);                                       // 设置CAN ID
void imu_set_mst_id(uint8_t mst_id);                                       // 设置主机ID
void imu_save_parameters();                                                // 保存参数
void imu_restore_settings();                                               // 恢复出厂设置

void imu_request_accel();                                                  // 请求加速度数据
void imu_request_gyro();                                                   // 请求角速度数据
void imu_request_euler();                                                  // 请求欧拉角数据
void imu_request_quat();                                                   // 请求四元数数据
void IMU_UpdateData(uint8_t* pData);									   // 更新IMU数据

#endif