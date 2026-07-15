/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "protocol.h"
#include "string.h"
#include "motor_el05.h"
#include "imu.h"
#include <stdint.h>
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 12;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 6;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  // 配置FDCAN1过滤�????????????
  FDCAN_FilterTypeDef sFilterConfig1 = {0};
  sFilterConfig1.IdType = FDCAN_EXTENDED_ID;          // 扩展ID
  sFilterConfig1.FilterIndex = 0;                     // 过滤�?????????????0
  sFilterConfig1.FilterType = FDCAN_FILTER_MASK;      // 掩码模式
  sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 进FIFO0
  sFilterConfig1.FilterID1 = 0x00000000;               // 接收�?????????????有ID
  sFilterConfig1.FilterID2 = 0x00000000;               // 掩码�?????????????0
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig1) != HAL_OK)
  {
    Error_Handler();
  }
  // �????????????活FIFO0接收中断（可选）
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // 启动FDCAN
  HAL_FDCAN_Start(&hfdcan1);
  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 7;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.MessageRAMOffset = 800;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 12;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 6;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  // 配置FDCAN2过滤�?????????????
  FDCAN_FilterTypeDef sFilterConfig2 = {0};
  sFilterConfig2.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig2.FilterIndex = 0;
  sFilterConfig2.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig2.FilterID1 = 0x00000000;
  sFilterConfig2.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig2) != HAL_OK)
  {
    Error_Handler();
  }
  // �?????????????活FIFO0接收中断（可选）
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // 启动FDCAN
  HAL_FDCAN_Start(&hfdcan2);
  /* USER CODE END FDCAN2_Init 2 */

}
/* FDCAN3 init function */
void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = ENABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 8;
  hfdcan3.Init.NominalSyncJumpWidth = 4;
  hfdcan3.Init.NominalTimeSeg1 = 7;
  hfdcan3.Init.NominalTimeSeg2 = 2;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.MessageRAMOffset = 1600;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 12;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 6;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */
  // 配置FDCAN3过滤�?????????????
  FDCAN_FilterTypeDef sFilterConfig3 = {0};
  sFilterConfig3.IdType = FDCAN_STANDARD_ID;          // 标准ID
  sFilterConfig3.FilterIndex = 0;                     // 过滤????0
  sFilterConfig3.FilterType = FDCAN_FILTER_MASK;      // 掩码模式
  sFilterConfig3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 进FIFO0
  sFilterConfig3.FilterID1 = 0x000;               // 接收????有ID
  sFilterConfig3.FilterID2 = 0x000;               // 掩码????0
  if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig3) != HAL_OK)
  {
    Error_Handler();
  }
  // �?????????????活FIFO0接收中断（可选）
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // 启动FDCAN
  HAL_FDCAN_Start(&hfdcan3);
  /* USER CODE END FDCAN3_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 3, 1);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspInit 0 */

  /* USER CODE END FDCAN3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN3 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**FDCAN3 GPIO Configuration
    PF6     ------> FDCAN3_RX
    PF7     ------> FDCAN3_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_FDCAN3;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* FDCAN3 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
  /* USER CODE BEGIN FDCAN3_MspInit 1 */

  /* USER CODE END FDCAN3_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN3)
  {
  /* USER CODE BEGIN FDCAN3_MspDeInit 0 */

  /* USER CODE END FDCAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN3 GPIO Configuration
    PF6     ------> FDCAN3_RX
    PF7     ------> FDCAN3_TX
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6|GPIO_PIN_7);

    /* FDCAN3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN3_IT0_IRQn);
  /* USER CODE BEGIN FDCAN3_MspDeInit 1 */

  /* USER CODE END FDCAN3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
// FDCAN发�?�扩展帧：id=29位扩展ID，data=数据，len=长度(1-8)
uint8_t FDCAN_Send_ExtFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef tx_header;
  if(len > 8 || len < 1) return 1;

  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_EXTENDED_ID;       // 扩展ID
  tx_header.TxFrameType = FDCAN_DATA_FRAME;   // 数据�?????????
  tx_header.DataLength = len;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;    // 无波特率切换
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;    // 普�?�CAN模式
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) == HAL_OK)
  {
    return 0;
  }
  return 1;
}

// FDCAN轮询接收扩展帧：返回0成功，id=接收到的扩展ID，data=数据，len=长度
// uint8_t FDCAN_Recv_ExtFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t *id, uint8_t *data, uint8_t *len)
// {
//   FDCAN_RxHeaderTypeDef rx_header;

//   if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK)
//   {
//     if(rx_header.IdType == FDCAN_EXTENDED_ID)  // 只处理扩展帧
//     {
//       *id = rx_header.Identifier;
//       *len = rx_header.DataLength;
//       return 0;
//     }
//   }
//   return 1;
// }

// FDCAN FIFO0 中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t data[8];
  FDCAN_RxHeaderTypeDef rx_header;
  
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    while(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) == HAL_OK)
    {
      if(rx_header.IdType == FDCAN_EXTENDED_ID)  // 只处理扩展ID
      {
        uint8_t id = (uint8_t)((rx_header.Identifier >> 8) & 0xFF); // 提取电机ID（假设在ID的特定位�???????????
        // 解析电机状�??
        //EL05_Parse_Status(rx_header.Identifier, data, &motor_status);
         // 解析位置(rad)：data[0-1]，16位无符号转浮点数
        uint16_t u_pos = (data[0] << 8) | data[1];
        // 解析速度(rad/s)：data[2-3]
        uint16_t u_vel = (data[2] << 8) | data[3];
        if (hfdcan->Instance == FDCAN1) {
          // 处理FDCAN1接收到的电机状�??
          if (id >= r_leg_pitch && id <= r_ankle_roll) {
          MotorIMU_Packet_float[(id-r_leg_pitch)*2+12]= (float)u_pos * (P_MAX - P_MIN) / 65535.0f + P_MIN;
          MotorIMU_Packet_float[(id-r_leg_pitch)*2+1+12]= (float)u_vel * (V_MAX - V_MIN) / 65535.0f + V_MIN;
          memcpy(&motor_status_buf[(id-r_leg_pitch)*4], data, 4); // 将接收到的数据复制到对应电机的状态缓冲区
          //memcpy(&MotorIMU_Packet_t[(id-r_leg_pitch)*4], data, 4); // 将接收到的数据复制到对应电机的状态缓冲区
          motor_status_ready |= (1 << (id - r_leg_pitch)); // 设置对应电机的数据就绪标�???
          motor_status_fault = (motor_status_fault & ~(1 << (id - r_leg_pitch))) | (((rx_header.Identifier >> 16) & 0x3F) != 0) << (id - r_leg_pitch);
          motor_status_mode = (motor_status_mode & ~(1 << (id - r_leg_pitch))) | ((((rx_header.Identifier >> 22) & 0x03) == 2) << (id - r_leg_pitch));
          }
          // 可以在这里添加代码将motor_status发到上位机，例如通过UART
        } else if (hfdcan->Instance == FDCAN2) {
          // 处理FDCAN2接收到的数据
          if (id >= l_leg_pitch && id <= l_ankle_roll) {
          MotorIMU_Packet_float[(id-l_leg_pitch)*2]= (float)u_pos * (P_MAX - P_MIN) / 65535.0f + P_MIN;
          MotorIMU_Packet_float[(id-l_leg_pitch)*2+1]= (float)u_vel * (V_MAX - V_MIN) / 65535.0f + V_MIN;
          memcpy(&motor_status_buf[(id-l_leg_pitch)*4+24], data, 4); // 将接收到的数据复制到对应电机的状态缓冲区
          //memcpy(&MotorIMU_Packet_t[(id-l_leg_pitch)*4+24], data, 4); // 将接收到的数据复制到对应电机的状态缓冲区
          motor_status_ready |= (1 << (id - l_leg_pitch + 6)); // 设置对应电机的数据就绪标??
          motor_status_fault = (motor_status_fault & ~(1 << (id - l_leg_pitch + 6))) | (((rx_header.Identifier >> 16) & 0x3F) != 0) << (id - l_leg_pitch + 6);
          motor_status_mode = (motor_status_mode & ~(1 << (id - l_leg_pitch + 6))) | ((((rx_header.Identifier >> 22) & 0x03) == 2) << (id - l_leg_pitch + 6));
          motor_fault_test = (motor_fault_test & ~(0X3F << (id - l_leg_pitch)*6)) | ((rx_header.Identifier >> 16) & 0x3F)<< (id - l_leg_pitch)*6;
          }
        }
      }else if(rx_header.IdType == FDCAN_STANDARD_ID) {
        // 处理IMU返回的数�?????????
        if (rx_header.Identifier == imu.mst_id) {
          IMU_UpdateData(data);
        }
      }
   }
  }
}

/* USER CODE END 1 */