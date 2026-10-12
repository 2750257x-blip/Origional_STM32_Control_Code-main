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
#include "stm32h7xx_hal.h"
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
#include "jetson_protocol.h"
#include "jetson_usb_cdc.h"
#include "QRCode_State.h"
// #include "lcd_spi_154.h"  // LCD hardware is not used on this robot.
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    ROBOT_STATE_IDLE = 0,    // 正常运行：上位机正常控电机上传IMU数据
    ROBOT_STATE_RHAND,          // 举右手
    ROBOT_STATE_LHAND,          // 举左手
    ROBOT_STATE_BOTHH,          // 举双手
    ROBOT_STATE_HEAD,           // 摇头
    ROBOT_STATE_LLEG,
    ROBOT_STATE_RLEG,
    ROBOT_STATE_LEG_STAND,
    ROBOT_STATE_CROSS,           // 跨棍
    ROBOT_STATE_RESET,            // 测试状态
    ROBOT_STATE_TEST_IMU,         // 测试状态
    ROBOT_STATE_TEST_UART,            // 测试状态
    ROBOT_STATE_TEST_INIT,            // 测试状态
}Robot_StateTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- 动作节拍：TIM2 中断每 10 ms 累加一次，替代动作里的 HAL_Delay ---- */
#define ACTION_TICK_MS   10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Robot_StateTypeDef robot_state = ROBOT_STATE_IDLE;   //  \o/  \o/  \o/  \o/  \o/  
uint8_t button_pressed = 0;
uint8_t button_pressed_last = 0;
uint16_t action_timer = 0;

uint8_t motor_control_buf[120] = {0};
float_t motor_control_buf_float[24] = {0};

uint8_t usb_tx_buffer[120] = {0};

uint8_t action_count = 0;
uint8_t action_count_finished = 0;

/* 状态帧发送节流：距上次成功发送满 g_state_send_interval_ms 才发下一帧。
 * 不再用 imu_data_ready 做门控——IMU 掉数据/掉电时那个锁存永远凑不齐，
 * 门控恒假会导致一帧都不发、Nano 100ms 超时掉线。可 ST-Link 实时改。 */
volatile uint32_t g_state_send_interval_ms = 20U;
volatile uint32_t g_state_send_last_ms = 0U;

/* ---- 断联定位诊断（只读，改了不影响控制行为）----
 * 断联当下用 SWD 读这几个量即可分辨原因：
 *   g_main_loop_count 不涨            -> CPU 卡住：三个 while(1) 之一，或 fault
 *   g_main_loop_max_dt_ms >= 100      -> 主循环被某个阻塞段堵过（HAL_Delay / Action_Goto）
 *   g_state_send_busy_streak 持续涨   -> SendState 一直 BUSY（TxState 卡死），设备在总线上但不发 */
volatile uint32_t g_main_loop_count = 0U;
volatile uint32_t g_main_loop_max_dt_ms = 0U;
volatile uint32_t g_state_send_ok_count = 0U;
volatile uint32_t g_state_send_fail_count = 0U;
volatile uint32_t g_state_send_busy_streak = 0U;


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

/* Nonblocking interpolation stays in the existing main-loop state machine. */
static float goto_start[12], goto_target[12];
static uint32_t goto_start_ms, goto_last_send_ms, goto_duration_ms;
static uint8_t goto_busy, local_watchdog_required, local_release_pending;
static uint32_t local_release_ms;
volatile uint8_t g_debug_local_leg_active = 0U;
/* Original PA4 / PC0 / PC13 poses, right motors followed by left motors. */
static const float leg_stand_pose[12] = {
    -.05f, -.25f, 0, -.2f, -.2f, -.25f, .05f, .25f, 0, .2f, .2f, .25f
};
static const float left_lift_pose[12] = {
    1.3f, 0, 0, -1.6f, -.1f, 0, 0, .4f, 0, 0, 0, .5f
};
static const float right_lift_pose[12] = {
    0, -.4f, 0, 0, 0, -.5f, -1.3f, 0, 0, 1.6f, .1f, 0
};

volatile uint32_t action_tick = 0;        /* TIM2 中断里累加 */
static uint8_t    action_step = 0;        /* 当前动作走到第几步 */
static uint32_t   action_step_tick = 0;   /* 当前步骤的计时起点 */
static Robot_StateTypeDef prev_robot_state = ROBOT_STATE_IDLE;     // 上一个状态机状态，用于检测状态切换
static ActionStatusPayload pending_action_status;                  //  上一个动作的状态，用于发送给上位机
static uint8_t pending_action_status_valid = 0U;                   //  上一个动作的状态是否有效，用于发送给上位机
static uint32_t active_event_id = 0U;                  //  当前正在执行的动作事件 ID，用于发送给上位机
static uint8_t active_action_id = 0U;                  //  当前正在执行的动作 ID，用于发送给上位机
static uint8_t active_action_status = 0U;              //  当前正在执行的动作状态，用于发送给上位机

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


/* Local startup/Action_Goto gains only; Jetson commands carry their own PD. */
static const float kp_add = 1.2f;
static const float kd_add = 1.5f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Robot_State_Machine(void);

void BUTTON_CHANGE(void);
void Close_All_Old_Func(void);
void Action_Goto(float rangle1, float rangle2, float rangle3, float rangle4, float rangle5, float rangle6, float langle1, float langle2, float langle3, float langle4, float langle5, float langle6, uint8_t Goto_time);

static void Action_Goto_Service(void);
static void Action_Goto_Cancel(void);
static void ROBOT_LEG(void);

void ROBOT_Comms_Service(void);
void ROBOT_RHAND(void);
void ROBOT_LHAND(void);
void ROBOT_HEAD(void);
void ROBOT_RESET(void);
void ROBOT_BOTHH(void);
void ROBOT_CROSS(void);
void ROBOT_TEST_IMU(void);
void ROBOT_TEST_UART(void);
void ROBOT_TEST_INIT(void);

// void LCD_State_Machine(void);

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
  // MX_SPI6_Init();  // SPI6 was only used by the disconnected LCD.
  /* USER CODE BEGIN 2 */
  JetsonRobotBridge_Init();
  HAL_Delay(200);       // 电机上电延时
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
  HAL_Delay(20);
  /* Preserve gravity-referenced roll and pitch. Zeroing at each boot would
     make a tilted startup pose appear level and corrupt projected gravity. */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // 启动PWM信号输出（舵机控制）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    // 启动PWM信号输出（舵机控制）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);    // 启动PWM信号输出（舵机控制）
  // SPI_LCD_Init();  // LCD 已停用；所有运行时显示调用也必须保持注释。
  motor_enable();
  HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);   // 动作节拍时基（10 ms）
  // LCD_DisplayText(10, 10, "Mode: IDLE");
  // LCD_DisplayText(10, 34, "motor_r: ");
  // LCD_DisplayNumber(118, 34, 0, 6);
  // LCD_DisplayText(10, 58, "motor_l: ");
  // LCD_DisplayNumber(118, 58, 0, 6);
  // LCD_DisplayText(10, 82, "motor_state: ");
  // LCD_DisplayHex(166, 82, 0, 4);
  // LCD_DisplayText(10, 106, "motor_fault: ");
  // LCD_DisplayHex(166, 106, 0, 4);
  // LCD_DisplayText(10, 130, "imu_ready: ");
  // LCD_DisplayHex(142, 130, 0, 2);
  // LCD_DisplayText(10, 154, "cycle: ");
  // LCD_DisplayNumber(94, 154, 0, 2);
  // LCD_DisplayText(10, 178, "warning: ");
  // LCD_DisplayNumber(118, 178, 0, 4);
  // LCD_DisplayText(10, 202, "imu_n: ");
  // LCD_DisplayNumber(94, 202, imu_data_count, 8);

  for(int i=0; i<12; i++)
  {
    leg_control[i] = (float)((motor_status_buf[i*4] << 8) | motor_status_buf[i*4+1]) * (P_MAX - P_MIN) / 65535.0f + P_MIN;
  }

  //Action_Goto(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50);
  //Action_Goto(0.15f, 0.05f, 0.0f, -0.30f, -0.15f, 0.0f, -0.15f, -0.05f, 0.0f, 0.30f, 0.15f, 0.00f, 50);
  system_control_warning = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  imu_set_zero();                         // 清零四元数，使复位后姿态归零
  HAL_Delay(500);
  Action_Goto(0.15f, -0.14f, 0.0f, -0.30f, -0.15f, -0.14f, -0.15f, 0.14f, 0.0f, 0.30f, 0.15f, 0.14f, 50);
  uint32_t loop_last_ms = HAL_GetTick();
  while (1)
  {
    BUTTON_CHANGE();
    /* 主循环最长间隔（诊断，见 g_main_loop_*）*/
    {      
      uint32_t loop_now_ms = HAL_GetTick();
      uint32_t loop_dt_ms = loop_now_ms - loop_last_ms;
      loop_last_ms = loop_now_ms;
      if (loop_dt_ms > g_main_loop_max_dt_ms) g_main_loop_max_dt_ms = loop_dt_ms;
      g_main_loop_count++;
    }
    /* 通信和腿部闭环与当前动作无关，每轮都先服务一次 */
    ROBOT_Comms_Service();
    Action_Goto_Service();
    Robot_State_Machine();
    //LCD_State_Machine();
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

/* 记录当前步骤的计时起点 */
static void Action_Step_Begin(void)
{
  action_step_tick = action_tick;
}

/* 当前步骤是否已经等满 ms 毫秒 */
static uint8_t Action_Step_Elapsed(uint32_t ms)
{
  return ((uint32_t)(action_tick - action_step_tick) * ACTION_TICK_MS) >= ms;
}

void Robot_State_Machine(void)
{
    if (robot_state != prev_robot_state) {
        prev_robot_state = robot_state;
        action_step = 0U;
        Action_Step_Begin();
    }

    switch(robot_state)
    {
      // 状态0：日常电机+IMU+USB双向通信
      case ROBOT_STATE_IDLE:
      {
        //Servo_SetAngle(&htim1, TIM_CHANNEL_1, (uint8_t)(180-(-MotorIMU_Packet_float[12]/3+30)));
        //Servo_SetAngle(&htim1, TIM_CHANNEL_2, (uint8_t)(MotorIMU_Packet_float[0]/3+30));
        break;
      }
      // 状态1：举右手
      case ROBOT_STATE_RHAND:
      {
        ROBOT_RHAND();
        break;
      }
      // 状态2：举左手
      case ROBOT_STATE_LHAND:
      {
        ROBOT_LHAND();
        break;
      }
      // 状态4：摇头
      case ROBOT_STATE_HEAD:
      {
        ROBOT_HEAD();
        break;
      }
      case ROBOT_STATE_LLEG:
      case ROBOT_STATE_RLEG:
      case ROBOT_STATE_LEG_STAND:
        ROBOT_LEG();
        break;
      // 状态5：重置
      case ROBOT_STATE_RESET:
      {
        ROBOT_RESET();
        break;
      }
      // 状态3：举双手
      case ROBOT_STATE_BOTHH:
      {
        ROBOT_BOTHH();
        break;
      }

      // 状态：跨棍
      case ROBOT_STATE_CROSS:
      {
        ROBOT_CROSS();
        break;
      }

      // 状态8：测试IMU通信
      case ROBOT_STATE_TEST_IMU:
      {
        ROBOT_TEST_IMU();
        break;
      }

      // 状态9：测试UART通信
      case ROBOT_STATE_TEST_UART:
      {
        ROBOT_TEST_UART();
        break;
      }

      // 状态10：测试初始化
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


// 记录当前动作的状态，并标记为有效，等待发送给上位机
static void Queue_Action_Status(uint32_t event_id, uint8_t action_id, uint8_t status)
{
  pending_action_status.event_id = event_id;          // 记录当前动作的事件 ID
  pending_action_status.action_id = action_id;        // 记录当前动作的动作 ID
  pending_action_status.status = status;              // 记录当前动作的状态
  pending_action_status_valid = 1U;                   // 标记为有效，等待发送给上位机
}


// 处理上位机发来的动作请求，根据请求的动作 ID 切换状态机，并发送动作状态给上位机
static void Handle_Action_Request(const ActionRequestPayload *request)
{
  Robot_StateTypeDef next_state;

  if (request->event_id == active_event_id && active_event_id != 0U) {         // 如果当前动作的事件 ID 与上位机请求的事件 ID 相同，说明上位机重复发送了请求
    Queue_Action_Status(request->event_id, request->action_id,
        request->action_id == active_action_id ? active_action_status : ACTION_STATUS_INVALID);
    return;
  }
  /* ---- 姿态事件（前倾/恢复）：不占用 robot_state，所以不走下面的状态机切换
   *      和 IDLE 守卫——前倾要跨越后续动作持续存在，而动作本身会把
   *      robot_state 从 IDLE 改走。这里单独处理，直接回 ACCEPTED。 ---- */
  if (request->action_id == ACTION_ID_LEAN ||
      request->action_id == ACTION_ID_RESTORE) {
    if (request->event_id == 0U ||
        g_debug_jetson_control_active == 0U ||
        !Protocol_CommandIsFresh(HAL_GetTick(), 100U) ||
        g_debug_local_leg_active != 0U) {
      Queue_Action_Status(request->event_id, request->action_id, ACTION_STATUS_BUSY);
      return;
    }
    if (request->action_id == ACTION_ID_LEAN) {
      if (robot_state != ROBOT_STATE_IDLE) {          // 只在空闲姿态下允许前倾
        Queue_Action_Status(request->event_id, request->action_id, ACTION_STATUS_BUSY);
        return;
      }
      if (g_debug_feedback_freeze == 0U) {
        JetsonRobotBridge_CaptureFrozenState();       // 先抓"前倾前"的那一帧
        g_debug_feedback_freeze = 1U;
      }
      JetsonRobotBridge_SetLean(1U);                  // 再改目标角度
    } else {
      JetsonRobotBridge_SetLean(0U);
      g_debug_feedback_freeze = 0U;
    }
    active_event_id = request->event_id;
    active_action_id = request->action_id;
    active_action_status = ACTION_STATUS_ACCEPTED;
    Queue_Action_Status(active_event_id, active_action_id, active_action_status);
    return;
  }

  switch (request->action_id) {                                // 根据上位机请求的动作 ID 切换状态机
    case 1U: next_state = ROBOT_STATE_LHAND; break;
    case 2U: next_state = ROBOT_STATE_RHAND; break;
    case 3U: next_state = ROBOT_STATE_LLEG; break;
    case 4U: next_state = ROBOT_STATE_RLEG; break;
    case 5U: next_state = ROBOT_STATE_BOTHH; break;
    case 6U: next_state = ROBOT_STATE_HEAD; break;
    default:
      Queue_Action_Status(request->event_id, request->action_id, ACTION_STATUS_INVALID);
      return;
  }
  if (request->event_id == 0U) {                           // 如果上位机请求的事件 ID 为 0，说明上位机没有正确设置事件 ID，返回无效状态
    Queue_Action_Status(request->event_id, request->action_id, ACTION_STATUS_INVALID);
    return;
  }
  if (robot_state != ROBOT_STATE_IDLE || g_debug_local_leg_active != 0U ||
      g_debug_jetson_control_active == 0U ||
      !Protocol_CommandIsFresh(HAL_GetTick(), 100U) ||
      active_action_status == ACTION_STATUS_ACCEPTED) {                                  // 如果当前状态机不在空闲状态，或者上位机控制未激活，或者上位机命令不新鲜，或者当前动作还在执行中，返回忙碌状态
    Queue_Action_Status(request->event_id, request->action_id, ACTION_STATUS_BUSY);
    return;
  }
  active_event_id = request->event_id;
  active_action_id = request->action_id;
  active_action_status = ACTION_STATUS_ACCEPTED;
  robot_state = next_state;
  action_count++;
  Queue_Action_Status(active_event_id, active_action_id, active_action_status);
}

/* 与上位机的USB通信 + 腿部闭环。必须在所有状态下都跑：一旦停下，Nano指令接收和状态回传就断了，
 * 100 ms看门狗一超时就会 stop_all_motors()，腿上的闭环整个掉。 */
void ROBOT_Comms_Service(void)
{
  // 处理通过USB CDC收到并经过CRC校验的Nano关节目标
  uint32_t invalid_command_count = g_debug_invalid_command_count;
  JetsonRobotBridge_ProcessCommand();
  if (g_debug_local_leg_active != 0U && g_debug_jetson_control_active != 0U) {
    local_watchdog_required = 1U;
  }
  /* A local trajectory must never keep commanding motors after host loss. */
  if (g_debug_local_leg_active != 0U &&
      (invalid_command_count != g_debug_invalid_command_count ||
       (local_watchdog_required != 0U &&
        (!Protocol_CommandIsFresh(HAL_GetTick(), 100U) ||
         g_debug_jetson_control_active == 0U)))) {
    Action_Goto_Cancel();
    robot_state = ROBOT_STATE_IDLE;
    for (uint8_t i = 0U; i < 6U; ++i) {
      EL05_Motor_Stop(&hfdcan1, (uint8_t)(r_leg_pitch + i));
      EL05_Motor_Stop(&hfdcan2, (uint8_t)(l_leg_pitch + i));
    }
    g_debug_jetson_control_active = 0U;
  }

  // 处理通过USB CDC收到的动作请求
  if (active_action_status == ACTION_STATUS_ACCEPTED) {
    if (!Protocol_CommandIsFresh(HAL_GetTick(), 100U) ||
        g_debug_jetson_control_active == 0U) {
      robot_state = ROBOT_STATE_IDLE;
      Action_Goto_Cancel();
      Servo_SetAngle(&htim1, TIM_CHANNEL_1, 150);
      Servo_SetAngle(&htim1, TIM_CHANNEL_2, 30);
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 90);
      active_action_status = ACTION_STATUS_FAILED;
      Queue_Action_Status(active_event_id, active_action_id, active_action_status);
    } else if (robot_state == ROBOT_STATE_IDLE &&
               JetsonRobotBridge_LeanSettled() != 0U) {
      /* LEAN/RESTORE 不占 robot_state，光看 IDLE 会在收下的下一轮就报 DONE，
       * 而前倾斜坡还要 580ms —— 上位机"发 LEAN 等 DONE"就会在机身还在倒的
       * 时候开始读卡。1/2/5/6 那条路径不受影响：没做前倾时上面那个函数恒真。 */
      active_action_status = ACTION_STATUS_DONE;
      Queue_Action_Status(active_event_id, active_action_id, active_action_status);
    }
  }
  ActionRequestPayload action_request;
  if (Protocol_TakeActionRequest(&action_request)) {
    Handle_Action_Request(&action_request);
  }
  if (pending_action_status_valid != 0U &&
      JetsonUsbCdc_SendActionStatus(&pending_action_status) == USBD_OK) {
    if (local_release_pending == 1U &&
        pending_action_status.event_id == active_event_id &&
        pending_action_status.status == ACTION_STATUS_DONE) {
      local_release_pending = 2U;
      local_release_ms = HAL_GetTick();
    }
    pending_action_status_valid = 0U;
  }

  // if (imu_data_ready == 0x00) {
  //     imu_request_accel();
  // }
  // else if (imu_data_ready== 0x01) {      
  //   imu_request_gyro();
  // }
  // else if (imu_data_ready== 0x03) {      
  //   imu_request_quat();
  // }
  //汇总电机状态和IMU数据，使用与通信测试工程相同的帧协议上传Nano
  __disable_irq();
  uint16_t motor_status_ready_copy = motor_status_ready;
  __enable_irq();
  /* 发送触发改成纯时间节流：距上次成功发送满 g_state_send_interval_ms 就发。
   * 不再依赖 imu_data_ready / IMU_DataIsFresh 做门控——IMU 掉数据/掉电时
   * 那个锁存永远凑不齐，门控恒假会导致一帧都不发、Nano 100ms 超时掉线。
   * 帧里的 STATE_IMU_VALID 仍由 SendState() 依 imu_data_ready + 新鲜度计算，
   * 所以 IMU 坏了 Nano 看得出来，但链路本身不会断。 */
  uint32_t now_ms = HAL_GetTick();
  if (motor_status_ready_copy == 0x0FFF &&
      (uint32_t)(now_ms - g_state_send_last_ms) >= g_state_send_interval_ms) {
    if (JetsonRobotBridge_SendState() == USBD_OK) { // USB发送成功
      g_state_send_last_ms = now_ms;
      g_state_send_ok_count++;
      g_state_send_busy_streak = 0U;
      // imu_data_ready = 0;   // 不再清：门控已改成时间节流，清了也没用
    } else {
      g_state_send_fail_count++;
      g_state_send_busy_streak++;
    }
  }

  // 收到Nano发来的二维码动作指令，切换动作模式
  // if(action_state != ACTION_IDLE)
  // {
  //   robot_state = ROBOT_STATE_ACTION;
  //   LCD_DisplayText(10, 10, "Mode : ACTION");
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); 
  //   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  // }
  // if(system_control_warning >= 10 || motor_status_fault != 0 || motor_status_mode != 0x0FFF) {
  //   robot_state = ROBOT_STATE_ERROR; // 如果连续3个周期没有正常控制，切换到异常状态
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
}

void ROBOT_RHAND(void)
{
  switch (action_step)
  {
    case 0:                                   /* 举右手，保持 3.2 s */
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(10, 10, "Mode : RHAND");
      Servo_SetAngle(&htim1, TIM_CHANNEL_1, 30);
      Action_Step_Begin();
      action_step = 1U;
      break;

    case 1:
      if (!Action_Step_Elapsed(3500U)) return;
      robot_state = ROBOT_STATE_IDLE;
      Servo_SetAngle(&htim1, TIM_CHANNEL_1, 150);
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(0, 10, "Mode : IDLE");
      action_count_finished++;
      break;
  }
}

void ROBOT_LHAND(void)
{
  switch (action_step)
  {
    case 0:                                   /* 举左手，保持 3.2 s */
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(10, 10, "Mode : LHAND");
      Servo_SetAngle(&htim1, TIM_CHANNEL_2, 150);
      Action_Step_Begin();
      action_step = 1U;
      break;

    case 1:
      if (!Action_Step_Elapsed(3500U)) return;
      robot_state = ROBOT_STATE_IDLE;
      Servo_SetAngle(&htim1, TIM_CHANNEL_2, 30);
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(0, 10, "Mode : IDLE");
      action_count_finished++;
      break;
  }
}

void ROBOT_HEAD(void)
{
  switch (action_step)
  {
    case 0:                                   /* 摇头，来回 5 次后回中 */
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(10, 10, "Mode : HEAD");
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
      Action_Step_Begin();
      action_step = 1U;
      break;

    case 1:
      if (!Action_Step_Elapsed(600U)) return;
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 180);
      Action_Step_Begin();
      action_step = 2U;
      break;

    case 2:
      if (!Action_Step_Elapsed(600U)) return;
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
      Action_Step_Begin();
      action_step = 3U;
      break;

    case 3:
      if (!Action_Step_Elapsed(600U)) return;
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 180);
      Action_Step_Begin();
      action_step = 4U;
      break;

    case 4:
      if (!Action_Step_Elapsed(600U)) return;
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
      Action_Step_Begin();
      action_step = 5U;
      break;

    case 5:
      if (!Action_Step_Elapsed(600U)) return;
      Servo_SetAngle(&htim1, TIM_CHANNEL_3, 90);
      Action_Step_Begin();
      action_step = 6U;
      break;

    case 6:
      if (!Action_Step_Elapsed(200U)) return;
      robot_state = ROBOT_STATE_IDLE;
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(0, 10, "Mode : IDLE");
      action_count_finished++;
      break;
  }
}

void ROBOT_RESET(void)
{
  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET){
    motor_setzero();
    HAL_Delay(200);
    }
}

void ROBOT_BOTHH(void)
{
  switch (action_step)
  {
    case 0:                                   /* 举双手，保持 3.2 s */
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(10, 10, "Mode : BOTHH");
      Servo_SetAngle(&htim1, TIM_CHANNEL_1, 30);
      Servo_SetAngle(&htim1, TIM_CHANNEL_2, 150);
      Action_Step_Begin();
      action_step = 1U;
      break;

    case 1:
      if (!Action_Step_Elapsed(3500U)) return;
      robot_state = ROBOT_STATE_IDLE;
      Servo_SetAngle(&htim1, TIM_CHANNEL_1, 150);
      Servo_SetAngle(&htim1, TIM_CHANNEL_2, 30);
      // LCD_ClearRect(10, 10, 240, 24);
      // LCD_DisplayText(0, 10, "Mode : IDLE");
      action_count_finished++;
      break;
  }
}

void ROBOT_CROSS(void)
{
  if (action_step == 0U) {
    Action_Goto(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 50U);
    action_step = 1U;
  } else if (goto_busy == 0U) {
    Action_Goto_Cancel();
    robot_state = ROBOT_STATE_IDLE;
  }
}

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
    
    //发送IMU数据
    __disable_irq();
    uint8_t imu_data_ready_copy = imu_data_ready; 
    memcpy(&usb_tx_buffer[48], imu_buf, sizeof(imu_buf)); 
    __enable_irq();
    if (imu_data_ready_copy == 0x03) { // 假设IMU数据都就绪
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
        // 这里可以添加针对未就绪电机的处理逻辑，例如重置电机状态并重新发送控制命令等
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
  static uint8_t previous_buttons;
  uint8_t buttons = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET ? 1U : 0U)
                  | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET ? 2U : 0U)
                  | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET ? 4U : 0U);
  uint8_t pressed = buttons & ~previous_buttons;
  previous_buttons = buttons;
  (void)JetsonRobotBridge_ServiceStartupButton();
  if (robot_state != ROBOT_STATE_IDLE || g_debug_local_leg_active != 0U ||
      active_action_status == ACTION_STATUS_ACCEPTED) return;
  if (pressed & 1U) robot_state = ROBOT_STATE_LLEG;
  else if (pressed & 2U) robot_state = ROBOT_STATE_RLEG;
  else if (pressed & 4U) {
    robot_state = ROBOT_STATE_LEG_STAND;
  }
}

void Close_All_Old_Func(void)
{
    // 1. 关闭定时器中断

    // 3. 关闭串口发送和接收
    HAL_UART_Abort(&huart1);
    HAL_UART_Abort(&huart2);

    // 5. 清空所有运行标志位
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
  /* ready 是12位掩码：bit0~5 右腿(FDCAN1)，bit6~11 左腿(FDCAN2)，按位拆开显示，1=该电机已上报 */
  // LCD_DisplayBinary(118, 34, motor_status_ready >>6, 6);          /* 右腿 bit5~bit0 */
  // LCD_DisplayBinary(118, 58, motor_status_ready, 6);     /* 左腿 bit11~bit6 */
  // LCD_DisplayHex(166, 82, motor_status_mode, 4);
  // LCD_DisplayHex(166, 106, motor_status_fault, 4);
  // LCD_DisplayHex(142, 130, (uint16_t)imu_data_ready, 2);
  // LCD_DisplayNumber(94, 154, (uint32_t)action_count, 2);
  // LCD_DisplayNumber(142, 154, (uint32_t)action_count_finished, 2);
  // LCD_DisplayNumber(118, 178, (uint32_t)imu_warning, 4);
  // LCD_DisplayNumber(94, 202, imu_data_count, 8);
}

void Action_Goto(float rangle1, float rangle2, float rangle3, float rangle4, float rangle5, float rangle6, float langle1, float langle2, float langle3, float langle4, float langle5, float langle6, uint8_t Goto_time)
{
  if (Goto_time == 0U) return;
  const float target[12] = {rangle1, rangle2, rangle3, rangle4, rangle5, rangle6,
                           langle1, langle2, langle3, langle4, langle5, langle6};
  memcpy(goto_start, leg_control, sizeof(goto_start));
  memcpy(goto_target, target, sizeof(goto_target));
  goto_start_ms = goto_last_send_ms = HAL_GetTick();
  goto_duration_ms = (uint32_t)Goto_time * 20U;
  goto_busy = g_debug_local_leg_active = 1U;
  local_watchdog_required = g_debug_jetson_control_active;
}

static void Action_Goto_Pose(const float target[12], uint8_t steps)
{
  Action_Goto(target[0], target[1], target[2], target[3], target[4], target[5],
              target[6], target[7], target[8], target[9], target[10], target[11], steps);
}

static void Action_Goto_Cancel(void)
{
  goto_busy = 0U;
  g_debug_local_leg_active = 0U;
  local_watchdog_required = 0U;
  local_release_pending = 0U;
}

/* One motor batch every 20 ms, including the three-second hold. */
static void Action_Goto_Service(void)
{
  uint32_t now_ms = HAL_GetTick();
  if (g_debug_local_leg_active == 0U || (uint32_t)(now_ms - goto_last_send_ms) < 20U) return;
  goto_last_send_ms = now_ms;
  if (goto_busy != 0U) {
    uint32_t elapsed = now_ms - goto_start_ms;
    float fraction = (float)elapsed / (float)goto_duration_ms;
    for (uint8_t i = 0U; i < 12U; ++i)
      leg_control[i] = elapsed >= goto_duration_ms ? goto_target[i]
                      : goto_start[i] + (goto_target[i] - goto_start[i]) * fraction;
    if (elapsed >= goto_duration_ms) goto_busy = 0U;
  }
  static const float kp[6] = {35, 30, 20, 35, 30, 12};
  static const float kd[6] = {1.5f, 1.2f, 1, 1.5f, 1.6f, .7f};
  for (uint8_t i = 0U; i < 6U; ++i) {
    EL05_Motor_Ctrl(&hfdcan1, (uint8_t)(r_leg_pitch + i), 0, leg_control[i], 0, kp[i]*kp_add, kd[i]*kd_add);
    EL05_Motor_Ctrl(&hfdcan2, (uint8_t)(l_leg_pitch + i), 0, leg_control[i+6U], 0, kp[i]*kp_add, kd[i]*kd_add);
  }
  system_control_cycle++;
  /* Give the host time to receive DONE and seed its next target from feedback. */
  if (goto_busy == 0U && robot_state == ROBOT_STATE_IDLE &&
      (local_release_pending == 0U || (local_release_pending == 2U &&
       (uint32_t)(now_ms - local_release_ms) >= 100U))) Action_Goto_Cancel();
}

static void ROBOT_LEG(void)
{
  switch (action_step) {
    case 0: {
      uint8_t feedback[48];
      __disable_irq();
      memcpy(feedback, motor_status_buf, sizeof(feedback));
      __enable_irq();
      for (uint8_t i = 0U; i < 12U; ++i)
        leg_control[i] = (float)((feedback[i*4U] << 8) | feedback[i*4U+1U])
                        * (P_MAX - P_MIN) / 65535.0f + P_MIN;
      Action_Goto_Pose(leg_stand_pose, 25U);
      action_step = robot_state == ROBOT_STATE_LEG_STAND ? 4U : 1U;
      break;
    }
    case 1:
      if (goto_busy != 0U) return;
      Action_Goto_Pose(robot_state == ROBOT_STATE_LLEG ? left_lift_pose : right_lift_pose, 50U);
      action_step = 2U;
      break;
    case 2:
      if (goto_busy != 0U) return;
      goto_start_ms = HAL_GetTick(); /* Start the hold after the lift endpoint is sent. */
      action_step = 3U;
      break;
    case 3:
      if ((uint32_t)(HAL_GetTick() - goto_start_ms) < 3000U) return;
      Action_Goto_Pose(leg_stand_pose, 25U); /* 25 x 20 ms = 0.5 s return. */
      action_step = 4U;
      break;
    case 4:
      if (goto_busy != 0U) return;
      Action_Step_Begin();
      action_step = 5U;
      break;
    case 5:
      if (!Action_Step_Elapsed(300U)) return;
      if (active_action_status == ACTION_STATUS_ACCEPTED) local_release_pending = 1U;
      else Action_Goto_Cancel();
      robot_state = ROBOT_STATE_IDLE;
      action_count_finished++;
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        action_tick++;
    }

  /* 健康监测在所有状态下都要跑，否则动作期间 imu_warning 不更新，
   * IMU掉线恢复那段会一直卡在触发条件里反复执行 */
  if (htim->Instance == TIM3) {
    if (system_control_cycle == system_control_cycle_copy) {
        system_control_warning ++; // 如果周期计数没有增加，设置警告标志
    } else {
        system_control_warning = 0; // 周期正常，清除警告标志
    }
    system_control_cycle_copy = system_control_cycle;
    if (imu_data_count == imu_data_count_copy) {
        imu_warning ++; // 如果周期计数没有增加，设置警告标志
    } else {
        imu_warning = 0; // 周期正常，清除警告标志
    }
    imu_data_count_copy = imu_data_count;
  }

  /* Goto interpolation is serviced by the main loop. */
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
