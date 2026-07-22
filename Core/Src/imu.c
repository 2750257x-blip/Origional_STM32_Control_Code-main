#include "imu.h"
#include "protocol.h"
#include <string.h>

imu_t imu;
volatile uint8_t imu_data_ready = 0;
volatile uint32_t imu_last_accel_ms = 0U;
volatile uint32_t imu_last_gyro_ms = 0U;
volatile uint32_t imu_last_quaternion_ms = 0U;
uint8_t imu_buf[12] = {0};
volatile int32_t imu_data_count = 0;

static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void imu_init(uint8_t can_id,uint8_t mst_id,FDCAN_HandleTypeDef *hfdcan)
{
	imu.can_id=can_id;
	imu.mst_id=mst_id;
	imu.can_handle=hfdcan;
}


static void imu_send_cmd(uint8_t reg_id,uint8_t ac,uint32_t data)
{
	
	if(imu.can_handle==NULL)
		return;
	
	FDCAN_TxHeaderTypeDef tx_header3;
	
	uint8_t buf[8]={0xCC,reg_id,ac,0xDD,0,0,0,0};
	memcpy(buf+4,&data,4);
	
	tx_header3.DataLength=FDCAN_DLC_BYTES_8;
	tx_header3.IdType=FDCAN_STANDARD_ID;
	tx_header3.TxFrameType=FDCAN_DATA_FRAME;
	tx_header3.Identifier=imu.can_id;
	tx_header3.FDFormat=FDCAN_CLASSIC_CAN;
	tx_header3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header3.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										
	tx_header3.MessageMarker = 0x00; 			      

	if(HAL_FDCAN_GetTxFifoFreeLevel(imu.can_handle)>2)
	{
		HAL_FDCAN_AddMessageToTxFifoQ(imu.can_handle,&tx_header3,buf);
	}
}

//imu�
void imu_write_reg(uint8_t reg_id,uint32_t data)
{
	imu_send_cmd(reg_id,CMD_WRITE,data);
}
//imu霂餃�
void imu_read_reg(uint8_t reg_id)
{
	imu_send_cmd(reg_id,CMD_READ,0);
}
//�IMU
void imu_reboot()
{
	imu_write_reg(REBOOT_IMU,0);
}
//霈曄蔭�嗡�
void imu_set_zero()
{
	imu_write_reg(SET_ZERO,0);
}
//�漲�∪�
void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI,0);
}
//��箔貌�∪�
void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI,0);
}

//��縑蝡臬
void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM,(uint8_t)port);
}
//霈曄蔭銝餃璅∪�銝�株��箏辣�?
void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY,delay);
}
//��唬蜓�冽芋撘?
void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE,1);
}
//��啗窈瘙芋撘?
void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE,0);
}

//霈曄蔭CAN瘜Ｙ�?
void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD,(uint8_t)baud);
}
//霈曄蔭CAN ID
void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID,can_id);
}
//霈曄蔭銝餅ID
void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID,mst_id);
}
//�唳颲�嚗洹��/���堆�
void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM,0);
}
//�Ｗ��箏�霈曄蔭
void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING,0);
}

//霂瑟��漲�唳
void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}
//霂瑟�閫漲�唳
void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}
//霂瑟�甈扳�閫�?
void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}
//霂瑟����唳�?
void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}


//�湔�漲�唳
void IMU_UpdateAccel(uint8_t* pData)
{
	// imu_buf[0] = pData[3]; 
	// imu_buf[1] = pData[2];
	// imu_buf[2] = pData[5];
	// imu_buf[3] = pData[4];
	// imu_buf[4] = pData[7];
	// imu_buf[5] = pData[6];

	// MotorIMU_Packet_t[48] = pData[3]; 
	// MotorIMU_Packet_t[49] = pData[2];
	// MotorIMU_Packet_t[50] = pData[5];
	// MotorIMU_Packet_t[51] = pData[4];
	// MotorIMU_Packet_t[52] = pData[7];
	// MotorIMU_Packet_t[53] = pData[6];

	
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	MotorIMU_Packet_float[24]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	MotorIMU_Packet_float[25]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	MotorIMU_Packet_float[26]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	memcpy(imu.accel, &MotorIMU_Packet_float[24], sizeof(imu.accel));
	
}
//�湔閫漲�唳
void IMU_UpdateGyro(uint8_t* pData)
{
    // imu_buf[6] = pData[3];
	// imu_buf[7] = pData[2];
	// imu_buf[8] = pData[5];
	// imu_buf[9] = pData[4];
	// imu_buf[10] = pData[7];
	// imu_buf[11] = pData[6];

	// MotorIMU_Packet_t[54] = pData[3];
	// MotorIMU_Packet_t[55] = pData[2];
	// MotorIMU_Packet_t[56] = pData[5];
	// MotorIMU_Packet_t[57] = pData[4];
	// MotorIMU_Packet_t[58] = pData[7];
	// MotorIMU_Packet_t[59] = pData[6];

	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	MotorIMU_Packet_float[27]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	MotorIMU_Packet_float[28]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	MotorIMU_Packet_float[29]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	memcpy(imu.gyro, &MotorIMU_Packet_float[27], sizeof(imu.gyro));
}

//�湔甈扳�閫�?
void IMU_UpdateEuler(uint8_t* pData)
{
	// imu_buf[12] = pData[3];
	// imu_buf[13] = pData[2];
	// imu_buf[14] = pData[5];
	// imu_buf[15] = pData[4];
	// imu_buf[16] = pData[7];
	// imu_buf[17] = pData[6];

	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imu.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16);
	imu.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16);
	imu.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16);
}

//�湔���唳�?
void IMU_UpdateQuaternion(uint8_t* pData)
{
	uint16_t w = ((uint16_t)pData[1] << 6) | (((uint16_t)pData[2] & 0xFCU) >> 2);
	uint16_t x = (((uint16_t)pData[2] & 0x03U) << 12) | ((uint16_t)pData[3] << 4) | (((uint16_t)pData[4] & 0xF0U) >> 4);
	uint16_t y = (((uint16_t)pData[4] & 0x0FU) << 10) | ((uint16_t)pData[5] << 2) | (((uint16_t)pData[6] & 0xC0U) >> 6);
	uint16_t z = (((uint16_t)pData[6] & 0x3FU) << 8) | (uint16_t)pData[7];
	
	imu.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imu.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
	memcpy(&MotorIMU_Packet_float[30], imu.q, sizeof(imu.q));
} 

//�湔�唳嚗�桅縑蝐餃�靚撖孵���啣�堆�
void IMU_UpdateData(uint8_t* pData)
{

	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			imu_last_accel_ms = HAL_GetTick();
			imu_data_ready |= IMU_DATA_ACCEL_READY;
			break;
		case 2:
			IMU_UpdateGyro(pData);
			imu_last_gyro_ms = HAL_GetTick();
			imu_data_ready |= IMU_DATA_GYRO_READY;
			break;
		case 3:
			IMU_UpdateEuler(pData);
			imu_data_ready |= (1<<3); // 霈曄蔭�唳撠梁貌��
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			imu_last_quaternion_ms = HAL_GetTick();
			imu_data_ready |= IMU_DATA_QUATERNION_READY;
			break;
	}
}

uint8_t IMU_DataIsFresh(uint32_t now_ms, uint32_t maximum_age_ms)
{
	return ((now_ms - imu_last_accel_ms) <= maximum_age_ms) &&
	       ((now_ms - imu_last_gyro_ms) <= maximum_age_ms) &&
	       ((now_ms - imu_last_quaternion_ms) <= maximum_age_ms);
}
