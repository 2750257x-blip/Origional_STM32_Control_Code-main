#ifndef __IMU_H
#define __IMU_H

#include "stm32h7xx_hal.h"

#define ACCEL_CAN_MAX   (235.2f)        //�漲�嚗?24g嚗?1g=9.8m/s簡嚗?
#define ACCEL_CAN_MIN	(-235.2f)       
#define GYRO_CAN_MAX	(34.88f)        //閫漲�嚗?2000簞/s嚗?1簞/s=0.01745 rad/s嚗?
#define GYRO_CAN_MIN	(-34.88f) 
#define PITCH_CAN_MAX	(90.0f)         //憪踵��湛�簣90簞嚗?1簞=0.01745 rad嚗?
#define PITCH_CAN_MIN	(-90.0f)      
#define ROLL_CAN_MAX	(180.0f)        //憪踵��湛�簣180簞嚗?1簞=0.01745 rad嚗?
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)        //憪踵��湛�簣180簞嚗?1簞=0.01745 rad嚗?
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN		(0.0f)          //皜拙漲�嚗?0-60��  
#define TEMP_MAX		(60.0f)
#define Quaternion_MIN	(-1.0f)         //���啗��湛�-1�?1嚗?
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
	COM_USB=0,    // USB��銝脣
	COM_RS485,    // RS485銝脣
	COM_CAN,      // CAN�餌瑪
	COM_VOFA      // VOFA�瑪�縑

}imu_com_port_e;

typedef enum
{
	CAN_BAUD_1M=0,    // CAN瘜Ｙ�?1Mbps
	CAN_BAUD_500K,    // CAN瘜Ｙ�?500Kbps
	CAN_BAUD_400K,    // CAN瘜Ｙ�?400Kbps
	CAN_BAUD_250K,    // CAN瘜Ｙ�?250Kbps
	CAN_BAUD_200K,    // CAN瘜Ｙ�?200Kbps
	CAN_BAUD_100K,    // CAN瘜Ｙ�?100Kbps
	CAN_BAUD_50K,     // CAN瘜Ｙ�?50Kbps
	CAN_BAUD_25K      // CAN瘜Ｙ�?25Kbps

}imu_baudrate_e;

typedef enum 
{
	REBOOT_IMU=0,              //�IMU                  1
	ACCEL_DATA,                //霂瑟��漲�唳            0
	GYRO_DATA,                 //霂瑟�閫漲�唳            0
	EULER_DATA,                //霂瑟�甈扳�閫�?            0
	QUAT_DATA,                 //霂瑟����唳�?            0

	SET_ZERO,                  //閫漲霈曄蔭�嗡�              1
	ACCEL_CALI,                //�漲�∪�                1
	GYRO_CALI,                 //��箔貌�∪�                1
	MAG_CALI,                  //蝤�霈⊥�?                1

	CHANGE_COM,                //��縑蝡臬              01
	SET_DELAY,                 //霈曄蔭銝餃璅∪�銝�株��箏辣�? 01
	CHANGE_ACTIVE,             //�銝餃/霂瑟�璅∪�         01

	SET_BAUD,                  //霈曄蔭CAN瘜Ｙ�?             01
	SET_CAN_ID,                //霈曄蔭CAN ID                01
	SET_MST_ID,                //霈曄蔭銝餅ID                 01
	DATA_OUTPUT_SELECTION,     //�唳颲�嚗洹��/���堆� 01
	SAVE_PARAM=254,            //靽���圈��仃摮�?         1
	RESTORE_SETTING=255        //�Ｗ��箏�霈曄蔭                  	1
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

	float q[4];              //���唳�殷���嚗�嚗?

	float cur_temp;          //敶�皜拙漲嚗�雿���

}imu_t;

extern imu_t imu;
extern volatile uint8_t imu_data_ready;
extern volatile uint32_t imu_last_accel_ms;
extern volatile uint32_t imu_last_gyro_ms;
extern volatile uint32_t imu_last_quaternion_ms;
extern uint8_t imu_buf[12];

void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan);  // IMU���?
void imu_write_reg(uint8_t reg_id,uint32_t data);                          // IMU撖��典��?
void imu_read_reg(uint8_t reg_id);                                         // IMU撖��刻粉�?

void imu_reboot();                                                         // �IMU
void imu_set_zero();                                                       // 霈曄蔭�嗡�
void imu_accel_calibration();                                              // �漲�∪�
void imu_gyro_calibration();                                               // ��箔貌�∪�

void imu_change_com_port(imu_com_port_e port);                             // ��縑蝡臬
void imu_set_active_mode_delay(uint32_t delay);                            // 霈曄蔭銝餃璅∪�銝�株��箏辣�?
void imu_change_to_active();                                               // ��唬蜓�冽芋撘?
void imu_change_to_request();                                              // ��啗窈瘙芋撘?

void imu_set_baud(imu_baudrate_e baud);                                    // 霈曄蔭CAN瘜Ｙ�?
void imu_set_can_id(uint8_t can_id);                                       // 霈曄蔭CAN ID
void imu_set_mst_id(uint8_t mst_id);                                       // 霈曄蔭銝餅ID
void imu_save_parameters();                                                // 靽��
void imu_restore_settings();                                               // �Ｗ��箏�霈曄蔭

void imu_request_accel();                                                  // 霂瑟��漲�唳
void imu_request_gyro();                                                   // 霂瑟�閫漲�唳
void imu_request_euler();                                                  // 霂瑟�甈扳�閫�?
void imu_request_quat();                                                   // 霂瑟����唳�?
void IMU_UpdateData(uint8_t* pData);									   // �湔IMU�唳

uint8_t IMU_DataIsFresh(uint32_t now_ms, uint32_t maximum_age_ms);

#endif
