/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include "motor_el05.h"
#include <stdlib.h>
#include <string.h>
#include <usbd_cdc_if.h>
#include "imu.h"
#include "protocol.h"
#include "jetson_robot_bridge.h"
#include "QRCode_State.h"
#include "lcd_spi_154.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    ROBOT_STATE_IDLE = 0,    // 正常运行：上位机正常控电机上传IMU数据
    ROBOT_STATE_ACTION,          // 执行二维码识别对应动�?????
    ROBOT_STATE_HOLD,          // 动作完成 停留3�?????
    ROBOT_STATE_ERROR,           // 异常状�??
    ROBOT_STATE_RESET,            // 测试状�??
    ROBOT_STATE_TEST,            // 测试状�??
    ROBOT_STATE_TEST_ACTION,      // 测试状�??
    ROBOT_STATE_TEST_IMU,         // 测试状�??
    ROBOT_STATE_TEST_UART,            // 测试状�??
    ROBOT_STATE_TEST_INIT,            // 测试状�??
}Robot_StateTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Robot_StateTypeDef robot_state = ROBOT_STATE_IDLE;
uint8_t button_pressed = 0;
uint8_t button_pressed_last = 0;
uint16_t action_timer = 0;

uint8_t motor_control_buf[120] = {0};
float_t motor_control_buf_float[24] = {0};

uint8_t usb_tx_buffer[120] = {0};


volatile uint32_t system_control_cycle = 0;
volatile uint32_t system_control_cycle_copy = 0;
volatile uint16_t system_control_warning = 0;  
volatile int32_t imu_data_count_copy = 0;
volatile uint16_t imu_warning = 0;  

volatile uint32_t controldata1number = 0;
volatile uint8_t  controldata1ready = 0;
float leg_control[12] = {0.0f};

float walklength1 = 0.005f;
float walklength2 = 0.005f;

volatile uint8_t Goto_ready = 0;
volatile uint8_t Goto_number = 0;
volatile float Goto_walklength[12] = {0.0f};

uint8_t uart_rx_buf1[120] = {0};
uint8_t uart_rx_data1[120] = {0};
volatile uint8_t uart_count1 = 0;
volatile uint8_t uart_start1 = 0;
volatile uint8_t uart_ready1 = 0;
uint8_t uart_rx_buf2[120] = {0};
uint8_t uart_rx_data2[120] = {0};
volatile uint8_t uart_count2 = 0;
volatile uint8_t uart_start2 = 0;
volatile uint8_t uart_ready2 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Robot_State_Machine(void);

void BUTTON_CHANGE(void);
void Close_All_Old_Func(void);
void Action_Goto(float rangle1, float rangle2, float rangle3, float rangle4, float rangle5, float rangle6, float langle1, float langle2, float langle3, float langle4, float langle5, float langle6, uint8_t Goto_time);

void ROBOT_IDLE(void);    
void ROBOT_ACTION(void);
void ROBOT_HOLD(void);
void ROBOT_ERROR(void);
void ROBOT_RESET(void);
void ROBOT_TEST(void);
void ROBOT_TEST_ACTION(void);
void ROBOT_TEST_IMU(void);
void ROBOT_TEST_UART(void);
void ROBOT_TEST_INIT(void);

void LCD_State_Machine(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/









  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_FDCAN1_Init();
  MX_USB_DEVICE_Init();
  MX_FDCAN3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
  JetsonRobotBridge_Init();
  HAL_Delay(500);       // 电机上电延时
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  imu_init(0x0F, 0xFD, &hfdcan3);
  // ==== 配置主动模式 100Hz 输出 6轴+四元数 ====
  imu_change_to_request();                // 切到请求模式才能配置
  HAL_Delay(20);
  imu_set_active_mode_delay(10);          // 100Hz (10ms)
  imu_write_reg(DATA_OUTPUT_SELECTION, 1);// 1=四元数, 0=欧拉角（6轴始终输出）
  imu_save_parameters();                  // 保存到IMU内部Flash
  HAL_Delay(20);
  imu_change_to_active();                 // 切换到主动模式

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // 启动PWM信号输出（舵机控制）
  SPI_LCD_Init();			// SPI LCD屏幕初始�???
  motor_enable();
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim3);
  LCD_DisplayText(10, 10, "Mode: IDLE");
  LCD_DisplayText(10, 34, "motor_ready: ");
  LCD_DisplayHex(166, 34, 0, 4);
  LCD_DisplayText(10, 58, "motor_state: ");
  LCD_DisplayHex(166, 58, 0, 4);
  LCD_DisplayText(10, 82, "motor_fault: ");
  LCD_DisplayHex(166, 82, 0, 4);
  LCD_DisplayText(10, 106, "imu_ready: ");
  LCD_DisplayHex(142, 106, 0, 2);
  LCD_DisplayText(10, 130, "cycle: ");
  LCD_DisplayNumber(94, 130, 0, 6);
  LCD_DisplayText(10, 154, "warning: ");
  LCD_DisplayNumber(118, 154, 0, 4);

  for(int i=0; i<12; i++)
  {
    leg_control[i] = (float)((motor_status_buf[i*4] << 8) | motor_status_buf[i*4+1]) * (P_MAX - P_MIN) / 65535.0f + P_MIN;
  }

  //Action_Goto(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50);
  Action_Goto(0.15f, 0.0f, 0.0f, -0.30f, -0.15f, 0.0f, -0.15f, 0.0f, 0.0f, 0.30f, 0.15f, 0.00f, 50);
  system_control_warning = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(2000);
  while (1)
  {
    BUTTON_CHANGE(); 
    Robot_State_Machine();
    LCD_State_Machine();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Robot_State_Machine(void)
{
    switch(robot_state)
    {
      // 状�??1：正常模�????? 日常电机+IMU+USB双向通信
      case ROBOT_STATE_IDLE:
      {
        ROBOT_IDLE();
        break;
      }

      // 状�??2：执行二维码识别对应的预设动作，动作完成后切到停留模�?????
      case ROBOT_STATE_ACTION:
      {
        ROBOT_ACTION();
        break;
      }

      // 状�??3：非阻塞停留3秒，全程不卡200Hz主循环，3秒后切回正常模式
      case ROBOT_STATE_HOLD:
      {
        ROBOT_HOLD();
        break;
      }

      // 状�??4：异常状�????? 自动切回正常模式
      case ROBOT_STATE_ERROR:
      {
        ROBOT_ERROR();
        break;
      }

      // 状�??5：测试状�????? 通过按键触发，执行预设动�?????
      case ROBOT_STATE_RESET:
      {
        ROBOT_RESET();
        break;
      }

      // 状�??6：测试状�?????
      case ROBOT_STATE_TEST:
      {
        ROBOT_TEST();
        break;
      }

      // 状�??7：测试二维码识别对应的预设动�?????
      case ROBOT_STATE_TEST_ACTION:
      {
        ROBOT_TEST_ACTION();
        break;
      }

      // 状�??8：测试IMU通信
      case ROBOT_STATE_TEST_IMU:
      {
        ROBOT_TEST_IMU();
        break;
      }

      // 状�??9：测试UART通信
      case ROBOT_STATE_TEST_UART:
      {
        ROBOT_TEST_UART();
        break;
      }

      // 状�??10：测试初始化
      case ROBOT_STATE_TEST_INIT:
      {
        ROBOT_TEST_INIT();
        break;
      }

      default:
      {
          robot_state = ROBOT_STATE_IDLE;
          action_state = ACTION_IDLE;
          break;
      }
    }
}

void ROBOT_IDLE(void) 
{
  // 处理通过USB CDC收到并�?�过CRC校验的Nano关节目标�?
  JetsonRobotBridge_ProcessCommand();

  // if (imu_data_ready == 0x00) {
  //     imu_request_accel();
  // }
  // else if (imu_data_ready== 0x01) {      
  //   imu_request_gyro();
  // }
  // else if (imu_data_ready== 0x03) {      
  //   imu_request_quat();
  // }
  //汇�?�电机状态和IMU数据，使用与通信测试工程相同的帧协议上传Nano�?
  __disable_irq();
  uint16_t motor_status_ready_copy = motor_status_ready; 
  uint8_t imu_data_ready_copy = imu_data_ready; 
  __enable_irq();
  if (motor_status_ready_copy == 0x0FFF && imu_data_ready_copy == 0x07) { // 假设12个电机都就绪且IMU数据都就�?????
    if (JetsonRobotBridge_SendState() == USBD_OK) { // USB发�?�成�?
    __disable_irq();
   // motor_status_ready = 0; // 重置计数
    imu_data_ready = 0;
    system_control_cycle ++;
    __enable_irq();
    }
  }

  // 收到Nano发来的二维码动作指令，切换动作模�?????
  if(action_state != ACTION_IDLE)
  {
    robot_state = ROBOT_STATE_ACTION;
    LCD_DisplayText(10, 10, "Mode : ACTION");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  }
  // if(system_control_warning >= 10 || motor_status_fault != 0 || motor_status_mode != 0x0FFF) {
  //   robot_state = ROBOT_STATE_ERROR; // 如果连续3个周期没有正常控制，切换到异常状�?????
  //   LCD_ClearRect(10, 10, 240, 24);
  //   LCD_DisplayText(10, 10, "Mode : ERROR");
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  // }
  if(imu_warning >=2 && imu_warning <=100) {
  imu_change_to_request();                // 切到请求模式才能配置
  HAL_Delay(20);
  imu_set_active_mode_delay(10);          // 100Hz (10ms)
  imu_write_reg(DATA_OUTPUT_SELECTION, 1);// 1=四元数, 0=欧拉角（6轴始终输出）
  imu_save_parameters();                  // 保存到IMU内部Flash
  HAL_Delay(20);
  imu_change_to_active();                 // 切换到主动模式
  }
  if (imu_warning > 100) {
    imu_data_ready=0x07;
  }
}

void ROBOT_ACTION(void)
{
  QRCode_State_Machine();
  action_timer = HAL_GetTick();
  robot_state = ROBOT_STATE_HOLD;
  LCD_ClearRect(10, 10, 240, 24);
  LCD_DisplayText(10, 10, "Mode : HOLD");
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

void ROBOT_HOLD(void)
{
  // 3秒时间到，切回正常运行模式，清空二维码指�?????
  if(HAL_GetTick() - action_timer >= 3000)
  {
    robot_state = ROBOT_STATE_IDLE;
    action_state = ACTION_IDLE;
    LCD_ClearRect(10, 10, 240, 24);
    LCD_DisplayText(0, 10, "Mode : IDLE");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  }
}

void ROBOT_ERROR(void)
{
  // 处理异常状�?�下的数�?????
  for (int i = 0; i < 12; i++) {
    if(motor_status_fault & (1 << i)) {
        // 这里可以添加针对未就绪电机的处理逻辑，例如重置电机状态并重新发�?�控制命令等
        EL05_Motor_Clear_Fault(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
        EL05_Motor_Enable(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
    }
  }
    for (int i = 0; i < 12; i++) {
    if(!(motor_status_mode & (1 << i))) {
        // 这里可以添加针对未就绪电机的处理逻辑，例如重置电机状态并重新发�?�控制命令等
        EL05_Motor_Enable(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
    }
  }
    for (int i = 0; i < 12; i++) {
    if(!(motor_status_ready & (1 << i)) && system_control_warning >= 3) {
        // 这里可以添加针对未就绪电机的处理逻辑，例如重置电机状态并重新发�?�控制命令等
        EL05_Motor_Enable(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
    }
  } 
    robot_state = ROBOT_STATE_IDLE;
    LCD_ClearRect(10, 10, 240, 24);
    LCD_DisplayText(10, 10, "Mode : IDLE");
    system_control_warning = 0; // 重置警告计数
    motor_status_fault = 0; // 重置电机故障标志
    motor_status_mode = 0X0FFF; // 重置电机模式标志
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
}

void ROBOT_RESET(void)
{
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    motor_setzero();
    HAL_Delay(200);
    }
}

void ROBOT_TEST(void)
{
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    HAL_Delay(200);
    HAL_TIM_Base_Start_IT(&htim2); // 启动定时器中断（用于周期性任务）
  }
  if(controldata1ready == 1 && controldata1number == 1) {
    controldata1ready = 0; // 重置数据就绪标志
    controldata1number = 0;

    EL05_Motor_Ctrl(&hfdcan1, r_leg_pitch, 0.0f, leg_control[0], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan1, r_leg_roll, 0.0f, leg_control[1], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan1, r_leg_yaw, 0.0f, leg_control[2], 0.0f, 50.0f, 5.0f);
    
    EL05_Motor_Ctrl(&hfdcan2, l_leg_pitch, 0.0f, leg_control[6], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan2, l_leg_roll, 0.0f, leg_control[7], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan2, l_leg_yaw, 0.0f, leg_control[8], 0.0f, 50.0f, 5.0f);

  }
    
  if(controldata1ready == 1 && controldata1number == 0) {
    controldata1ready = 0; // 重置数据就绪标志
    controldata1number = 1;

    EL05_Motor_Ctrl(&hfdcan1, r_knee_pitch, 0.0f, leg_control[3], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan1, r_ankle_pitch, 0.0f, leg_control[4], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan1, r_ankle_roll, 0.0f, leg_control[5], 0.0f, 50.0f, 5.0f);

    EL05_Motor_Ctrl(&hfdcan2, l_knee_pitch, 0.0f, leg_control[9], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan2, l_ankle_pitch, 0.0f, leg_control[10], 0.0f, 50.0f, 5.0f);
    EL05_Motor_Ctrl(&hfdcan2, l_ankle_roll, 0.0f, leg_control[11], 0.0f, 50.0f, 5.0f);
  }
    
    //发�?�电机状�?????
    __disable_irq();
    uint16_t motor_status_ready_copy = motor_status_ready;
    memcpy(&usb_tx_buffer[0], motor_status_buf, sizeof(motor_status_buf));
    __enable_irq();
    if (motor_status_ready_copy == 0x3F) { 
      if (CDC_Transmit_HS(&usb_tx_buffer[0], sizeof(motor_status_buf)) == USBD_OK) {
        __disable_irq();
        motor_status_ready = 0;
        __enable_irq();
      }
    }
}

void ROBOT_TEST_ACTION(void)
{
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    HAL_Delay(200);
    Action_Goto(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50);
    }
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET){
    HAL_Delay(200);
    Action_Goto(1.3f, 0.0f, 0.0f, -1.6f, -0.1f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f, 0.0f, 0.5f, 50);
    }
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
    HAL_Delay(200);
    Action_Goto(0.15f, 0.0f, 0.0f, -0.30f, -0.15f, 0.0f, -0.15f, 0.0f, 0.0f, 0.30f, 0.15f, 0.00f, 50);
    }
}
//sk-f14f125467984078979ed8ef4f748cc5
void ROBOT_TEST_IMU(void)
{
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    HAL_Delay(200);
    memset(usb_tx_buffer, 0, sizeof(usb_tx_buffer));
    HAL_TIM_Base_Start_IT(&htim2); // 启动定时器中断（用于周期性任务）
  }
  if(controldata1ready == 1 && controldata1number == 1) {
    controldata1ready = 0; // 重置数据就绪标志
    controldata1number = 0;

    imu_request_accel();
  }
    
  if(controldata1ready == 1 && controldata1number == 0) {
    controldata1ready = 0; // 重置数据就绪标志
    controldata1number = 1;
    
    imu_request_gyro();
    }
    
    //发�?�IMU数据
    __disable_irq();
    uint8_t imu_data_ready_copy = imu_data_ready; 
    memcpy(&usb_tx_buffer[48], imu_buf, sizeof(imu_buf)); 
    __enable_irq();
    if (imu_data_ready_copy == 0x03) { // 假设IMU数据都就�?????
      if (CDC_Transmit_HS(&usb_tx_buffer[48], sizeof(imu_buf)+52) == USBD_OK) {
        __disable_irq();
        imu_data_ready = 0; // 重置计数
        __enable_irq();
      }
    }
}

void ROBOT_TEST_UART(void)
{
  if(uart_ready1) {
    HAL_UART_Transmit(&huart2, uart_rx_data1, 1, 100);
    Servo_SetAngle(&htim1, TIM_CHANNEL_1, uart_rx_data1[0]);
    uart_ready1 = 0;
  }
  if(uart_ready2) {
    HAL_UART_Transmit(&huart2, uart_rx_data2, 2, 100);

    if( uart_rx_data2[0] >= r_leg_pitch && uart_rx_data2[0] <= r_ankle_roll) {
      if(uart_rx_data2[1] == 0x00) {
        leg_control[uart_rx_data2[0]-r_leg_pitch] -= 0.1f;
      }if(uart_rx_data2[1] == 0x01) {
        leg_control[uart_rx_data2[0]-r_leg_pitch] += 0.1f;
      }
      EL05_Motor_Ctrl(&hfdcan1, uart_rx_data2[0], 0.0f, leg_control[uart_rx_data2[0]-r_leg_pitch], 0.0f, 50.0f, 5.0f);
    }

    if( uart_rx_data2[0] >= l_leg_pitch && uart_rx_data2[0] <= l_ankle_roll) {
      if(uart_rx_data2[1] == 0x00) {
        leg_control[uart_rx_data2[0]-l_leg_pitch+6] -= 0.1f;
      }if(uart_rx_data2[1] == 0x01) {
        leg_control[uart_rx_data2[0]-l_leg_pitch+6] += 0.1f;
      }
      EL05_Motor_Ctrl(&hfdcan2, uart_rx_data2[0], 0.0f, leg_control[uart_rx_data2[0]-l_leg_pitch+6], 0.0f, 50.0f, 5.0f);
    }
      
    uart_ready2 = 0;
  }
}

void ROBOT_TEST_INIT(void)
{
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    HAL_Delay(200);
      motor_enable();
    }
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET){
    HAL_Delay(200);
    for (int i = 0; i < 12; i++) {
    if(motor_status_fault & (1 << i)) {
        // 这里可以添加针对未就绪电机的处理逻辑，例如重置电机状态并重新发�?�控制命令等
        EL05_Motor_Clear_Fault(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
        EL05_Motor_Enable(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
      }
     }
    }
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
    HAL_Delay(200);
    for (int i = 0; i < 12; i++) {
      EL05_Motor_Stop(i < 6 ? &hfdcan1 : &hfdcan2, i < 6 ? i + r_leg_pitch : i - 6 + l_leg_pitch);
     }
    }
}

void BUTTON_CHANGE(void)
{
  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET || HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    
     if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_SET) {
        if(button_pressed == 6) {
          button_pressed = 0; // 防止溢出
        }
        else {
          button_pressed ++;
        }
      }
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET) {
        if(button_pressed == 0) {
          button_pressed = 6; // 防止溢出
        }
        else {
          button_pressed --;
        }
      }
      if(button_pressed == 0) {  // 按键0：正常模�?????
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : IDLE");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
      }
      else if(button_pressed == 1) {  // 按键1：设零位
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : RESET");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
      }
      else if(button_pressed == 2) {  // 按键2：测�?????
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : TEST");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
      }
      else if(button_pressed == 3) { //动作测试
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : TEST_ACTION");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
      }
      else if(button_pressed == 4) { // IMU测试
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : TEST_IMU");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
      }
      else if(button_pressed == 5) { // UART测试
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : TEST_UART");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
      }
      else if(button_pressed == 6) { // INIT测试
        LCD_ClearRect(10, 10, 240, 24);
        LCD_DisplayText(10, 10, "Mode : TEST_INIT");
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
      }
      HAL_Delay(1000);
    }

  if(button_pressed != button_pressed_last) {
    button_pressed_last = button_pressed;
    Close_All_Old_Func();
    switch(button_pressed)
    {
      case 0:
        robot_state = ROBOT_STATE_IDLE;
        break;
      case 1:
        robot_state = ROBOT_STATE_RESET;
        break;
      case 2:
        robot_state = ROBOT_STATE_TEST;
        Action_Goto(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50);
        break;
      case 3:
        robot_state = ROBOT_STATE_TEST_ACTION;
        break;
      case 4:
        robot_state = ROBOT_STATE_TEST_IMU; 
        break;
      case 5:
        robot_state = ROBOT_STATE_TEST_UART;
        HAL_UART_Receive_IT(&huart1, &uart_rx_buf1[uart_count1], 1);
        HAL_UART_Receive_IT(&huart2, &uart_rx_buf2[uart_count2], 1);
        break;
      case 6:
        robot_state = ROBOT_STATE_TEST_INIT;
        break;
      default:
        robot_state = ROBOT_STATE_IDLE;
        break;
    }
  }
}

void Close_All_Old_Func(void)
{
    // 1. 关闭定时器中�?????

    // 3. 关闭串口发�?�和接收
    HAL_UART_Abort(&huart1);
    HAL_UART_Abort(&huart2);

    // 5. 清空�?????有运行标志位
    system_control_cycle = 0;
    system_control_cycle_copy = 0;
    system_control_warning = 0;  

    controldata1number = 0;
    controldata1ready = 0;

    uart_count1 = 0;
    uart_start1 = 0;
    uart_ready1 = 0;
    uart_count2 = 0;
    uart_start2 = 0;
    uart_ready2 = 0;

    motor_status_ready = 0;
    imu_data_ready = 0;
}

void LCD_State_Machine(void)
{
  LCD_DisplayHex(166, 34, motor_status_ready, 4);
  LCD_DisplayHex(166, 58, motor_status_mode, 4);
  LCD_DisplayHex(166, 82, motor_status_fault, 4);
  LCD_DisplayHex(142, 106, (uint16_t)imu_data_ready, 2);
  LCD_DisplayNumber(94, 130, (uint32_t)system_control_cycle, 6);
  LCD_DisplayNumber(118, 154, (uint32_t)imu_warning, 4);
  LCD_DisplayHex(10, 178, motor_fault_test, 4);
  LCD_DisplayNumber(10, 202, imu_data_count, 8);
}

void Action_Goto(float rangle1, float rangle2, float rangle3, float rangle4, float rangle5, float rangle6, float langle1, float langle2, float langle3, float langle4, float langle5, float langle6, uint8_t Goto_time)
{
  Goto_walklength[0] = (rangle1 - leg_control[0]) / Goto_time;
  Goto_walklength[1] = (rangle2 - leg_control[1]) / Goto_time;
  Goto_walklength[2] = (rangle3 - leg_control[2]) / Goto_time;
  Goto_walklength[3] = (rangle4 - leg_control[3]) / Goto_time;
  Goto_walklength[4] = (rangle5 - leg_control[4]) / Goto_time;
  Goto_walklength[5] = (rangle6 - leg_control[5]) / Goto_time;
  Goto_walklength[6] = (langle1 - leg_control[6]) / Goto_time;
  Goto_walklength[7] = (langle2 - leg_control[7]) / Goto_time;
  Goto_walklength[8] = (langle3 - leg_control[8]) / Goto_time;
  Goto_walklength[9] = (langle4 - leg_control[9]) / Goto_time;
  Goto_walklength[10] = (langle5 - leg_control[10]) / Goto_time;
  Goto_walklength[11] = (langle6 - leg_control[11]) / Goto_time;
  HAL_TIM_Base_Start_IT(&htim4); // 启动定时器中断（用于周期性任务）
  while(Goto_number < Goto_time) {
    if(Goto_ready){
      Goto_ready = 0;
      EL05_Motor_Ctrl(&hfdcan1, r_leg_pitch, 0.0f, leg_control[0], 0.0f, 35.0f, 1.5f);
      EL05_Motor_Ctrl(&hfdcan1, r_leg_roll, 0.0f, leg_control[1], 0.0f, 30.0f, 1.2f);
      EL05_Motor_Ctrl(&hfdcan1, r_leg_yaw, 0.0f, leg_control[2], 0.0f, 20.0f, 1.0f);
      EL05_Motor_Ctrl(&hfdcan1, r_knee_pitch, 0.0f, leg_control[3], 0.0f, 35.0f, 1.5f);
      EL05_Motor_Ctrl(&hfdcan1, r_ankle_pitch, 0.0f, leg_control[4], 0.0f, 15.0f, 0.8f);
      EL05_Motor_Ctrl(&hfdcan1, r_ankle_roll, 0.0f, leg_control[5], 0.0f, 12.0f, 0.7f);
      EL05_Motor_Ctrl(&hfdcan2, l_leg_pitch, 0.0f, leg_control[6], 0.0f, 35.0f, 1.5f);
      EL05_Motor_Ctrl(&hfdcan2, l_leg_roll, 0.0f, leg_control[7], 0.0f, 30.0f, 1.2f);
      EL05_Motor_Ctrl(&hfdcan2, l_leg_yaw, 0.0f, leg_control[8], 0.0f, 20.0f, 1.0f);
      EL05_Motor_Ctrl(&hfdcan2, l_knee_pitch, 0.0f, leg_control[9], 0.0f, 35.0f, 1.5f);
      EL05_Motor_Ctrl(&hfdcan2, l_ankle_pitch, 0.0f, leg_control[10], 0.0f, 15.0f, 0.8f);
      EL05_Motor_Ctrl(&hfdcan2, l_ankle_roll, 0.0f, leg_control[11], 0.0f, 12.0f, 0.7f);
    }
  }
    HAL_TIM_Base_Stop_IT(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    Goto_ready = 0;
    Goto_number = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        controldata1ready = 1; // 设置数据就绪标志
        leg_control[0] += walklength1*1;                                //0  1  0  -1  0 
        leg_control[1] += -walklength2*0.2;                                //0  -1  0  -1  0
        leg_control[2] += walklength1*0;                                    //0  0  0  0  0
        leg_control[3] -= walklength1*0.3+walklength2*0.3;             //0  1  0  0  0
        leg_control[4] += walklength1*0.4;                              //0  1  0  -1  0
        leg_control[5] += -walklength2*0.2;                              //0  -1  0  -1  0

        leg_control[6] += walklength1*1;                                //0  1  0  -1  0
        leg_control[7] += walklength2*0.2;                                 //0  1  0  1  0
        leg_control[8] += walklength1*0;                                    //0  0  0  0  0
        leg_control[9] -= walklength1*0.3-walklength2*0.3;             //0  0  0  -1  0
        leg_control[10] += walklength1*0.4;                              //0  1  0  -1  0 
        leg_control[11] += walklength2*0.2;                               //0  1 _ankle_roll_control += walklength2*0.2;                               //0  1  

        if (leg_control[0] > 0.4f)
        {
            walklength1 = -walklength1; // 反转步长方向                         //0  -1  0  -1  0
            walklength2 = -walklength2;                                        //0   1  0   1  0
        } 
        else if (leg_control[0] < -0.4f)
        {
            walklength1 = -walklength1; // 反转步长方向
            walklength2 = -walklength2;
        }
        if (leg_control[0] <= 1e-6f)
        {
            walklength2 = -walklength2;
        }
    }

  if (htim->Instance == TIM3 && robot_state == ROBOT_STATE_IDLE) {
    if (system_control_cycle == system_control_cycle_copy) {
        system_control_warning ++; // 如果周期计数没有增加，设置警告标�?????
    } else {
        system_control_warning = 0; // 周期正常，清除警告标�?????
    }
    system_control_cycle_copy = system_control_cycle;
    if (imu_data_count == imu_data_count_copy) {
        imu_warning ++; // 如果周期计数没有增加，设置警告标?????
    } else {
        imu_warning = 0; // 周期正常，清除警告标?????
    }
    imu_data_count_copy = imu_data_count;
  }

  if (htim->Instance == TIM4) {
        Goto_ready = 1; // 设置数据就绪标志
        Goto_number++; // 增加 goto 计数
        leg_control[0] += Goto_walklength[0];                                
        leg_control[1] += Goto_walklength[1];                               
        leg_control[2] += Goto_walklength[2];                                  
        leg_control[3] += Goto_walklength[3];            
        leg_control[4] += Goto_walklength[4];                              
        leg_control[5] += Goto_walklength[5];                            

        leg_control[6] += Goto_walklength[6];                                
        leg_control[7] += Goto_walklength[7];                                
        leg_control[8] += Goto_walklength[8];                                   
        leg_control[9] += Goto_walklength[9];             
        leg_control[10] += Goto_walklength[10];                             
        leg_control[11] += Goto_walklength[11];                               
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
          if(uart_rx_buf1[uart_count1] == 0xff && uart_start1 == 0 ) {
              uart_count1 = 0;
              uart_start1 = 1;
          }
          else if(uart_rx_buf1[uart_count1] == 0xee && uart_start1 == 1) {
              uart_ready1 = 1; // 设置数据就绪标志
              uart_start1 = 0;
              memcpy(uart_rx_data1, uart_rx_buf1, uart_count1);
          }
          else if(uart_start1 == 1) {
              uart_count1++;
          }
          if(uart_count1 >= 120) {
              uart_count1 = 0;
              uart_start1 = 0;
          }
        // 重新启动 UART 接收中断
        HAL_UART_Receive_IT(&huart1, &uart_rx_buf1[uart_count1], 1);
    }
    if (huart->Instance == USART2) {
          if(uart_rx_buf2[uart_count2] == 0xff && uart_start2 == 0 ) {
              uart_start2 = 1;
          }
          if(uart_rx_buf2[uart_count2] == 0xfe && uart_start2 == 1 ) {
              uart_count2 = 0;
              uart_start2 = 2; 
          }
          else if(uart_rx_buf2[uart_count2] == 0xee && uart_start2 == 2) {
            if(uart_rx_buf2[(uart_count2 -1)] == 0xde){
              uart_ready2 = 1; 
              uart_start2 = 0;
              memcpy(uart_rx_data2, uart_rx_buf2, (uart_count2 -1)); 
            }
            else{
              uart_count2++;
            }
          }
          else if(uart_start2 == 2) {
              uart_count2++;
          }
          if(uart_count2 >= 120) {
              uart_count2 = 0;
              uart_start2 = 0;
          }
        // 重新启动 UART 接收中断
        HAL_UART_Receive_IT(&huart2, &uart_rx_buf2[uart_count2], 1);
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x2007E000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8KB;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
