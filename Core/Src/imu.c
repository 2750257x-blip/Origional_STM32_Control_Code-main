#include "imu.h"
#include "protocol.h"
#include <string.h>

imu_t imu;
volatile uint8_t imu_data_ready = 0;
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

//imu写入
void imu_write_reg(uint8_t reg_id,uint32_t data)
{
	imu_send_cmd(reg_id,CMD_WRITE,data);
}
//imu读取
void imu_read_reg(uint8_t reg_id)
{
	imu_send_cmd(reg_id,CMD_READ,0);
}
//重启IMU
void imu_reboot()
{
	imu_write_reg(REBOOT_IMU,0);
}
//设置零位
void imu_set_zero()
{
	imu_write_reg(SET_ZERO,0);
}
//加速度校准
void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI,0);
}
//陀螺仪校准
void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI,0);
}

//切换通信端口
void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM,(uint8_t)port);
}
//设置主动模式下数据输出延�?
void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY,delay);
}
//切换到主动模�?
void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE,1);
}
//切换到请求模�?
void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE,0);
}

//设置CAN波特�?
void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD,(uint8_t)baud);
}
//设置CAN ID
void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID,can_id);
}
//设置主机ID
void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID,mst_id);
}
//数据输出选择（欧拉角/四元数）
void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM,0);
}
//恢复出厂设置
void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING,0);
}

//请求加速度数据
void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}
//请求角速度数据
void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}
//请求欧拉角数�?
void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}
//请求四元数数�?
void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}


//更新加速度数据
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
	
}
//更新角速度数据
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
}

//更新欧拉角数�?
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

//更新四元数数�?
void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	MotorIMU_Packet_float[30] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	MotorIMU_Packet_float[31] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	MotorIMU_Packet_float[32] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	MotorIMU_Packet_float[33] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
} 

//更新数据（根据通信类型调用对应的更新函数）
void IMU_UpdateData(uint8_t* pData)
{

	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			imu_data_ready |= (1<<0); // 设置数据就绪标志
			break;
		case 2:
			IMU_UpdateGyro(pData);
			imu_data_ready |= (1<<1); // 设置数据就绪标志
			break;
		case 3:
			IMU_UpdateEuler(pData);
			imu_data_ready |= (1<<3); // 设置数据就绪标志
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			imu_data_ready |= (1<<2); // 设置数据就绪标志
			break;
	}
}