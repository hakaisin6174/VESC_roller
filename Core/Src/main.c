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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "can/can_cube.h"
#include "can/can_stm32.h"
#include "pid/pid.h"
#include "vesc/vesc_core.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  VESC_APP_STOPPED = 0,
  VESC_APP_STARTING_CURRENT,
  VESC_APP_STARTING_DUTY,
  VESC_APP_DRIVING,
  VESC_APP_BRAKING,
} VescAppState;

typedef struct {
  uint16_t previous_count;
  int64_t continuous_count;
  uint32_t last_update_tick;
  int32_t rpm;
  bool valid;
  bool fresh;
  bool rpm_updated;
  float rpm_dt_sec;
} RollerEncoderData;

typedef struct {
  int16_t input_voltage_x10;
  uint32_t last_update_tick;
  bool valid;
} VescStatus5Data;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 2台のVESCはCAN1に接続し、同じ電流/duty指令で同期させます。 */
#define VESC_MOTOR_1_CAN_ID            105U
#define VESC_MOTOR_2_CAN_ID            112U

/* VESC CAN_PACKET_STATUS_5: tachometer[4 byte] + input voltage x10[2 byte] */
#define VESC_CAN_PACKET_STATUS_5_ID      27U
#define VESC_STATUS_5_TIMEOUT_MS       1000U

/* 一時的なCAN送信メールボックス混雑を即時故障扱いしないための設定です。 */
#define VESC_CAN_TX_RETRY_TIMEOUT_MS       2U
#define VESC_CAN_TX_FAILURE_LIMIT          3U
#define VESC_CAN_TX_FAILURE_MAILBOX        1U
#define VESC_CAN_TX_FAILURE_HAL            2U

/*
 * kimura_rollerのM3508ローラー制御と同じ目標回転数です。
 * 逆方向へ回す場合は負の値へ変更してください。
 */
#define ROLLER_TARGET_RPM             -4500.0f

/*
 * 目標ローラーRPMから起動dutyを計算するための駆動系定数です。
 * duty = target_rpm / (Kv * battery_voltage * motor_gear / roller_gear)
 */
#define ROLLER_MOTOR_KV_RPM_PER_V        560.0f
#define ROLLER_MOTOR_GEAR_TEETH           13.0f
#define ROLLER_GEAR_TEETH                 25.0f
#define ROLLER_NOMINAL_BATTERY_V          24.5f
#define ROLLER_MIN_CALC_BATTERY_V          5.0f
#define ROLLER_DUTY_LOAD_COMPENSATION       1.14f

/* 補正電圧[V] = 平均電圧[V] * GAIN + OFFSET[V] */
#define ROLLER_BATTERY_VOLTAGE_GAIN        (0.83f / 0.85f)
#define ROLLER_BATTERY_VOLTAGE_OFFSET_V    0.92f

/*
 * 起動時は目標RPMをPIDへステップ入力しません。最初に小さい電流で
 * 確実に回転を始め、300RPM到達後にduty制御で目標付近まで加速します。
 * 電流とdutyの符号は目標RPMから決めるため、ここには絶対値を設定します。
 */
#define ROLLER_STARTUP_CURRENT_A           2.0f
#define ROLLER_STARTUP_CURRENT_RAMP_MS    150U
#define ROLLER_STARTUP_DUTY_SWITCH_RPM    300.0f
#define ROLLER_STARTUP_DUTY_BEGIN_ABS       0.05f
#define ROLLER_STARTUP_DUTY_MAX_ABS        0.95f
#define ROLLER_STARTUP_DUTY_RAMP_MS       1000U
#define ROLLER_STARTUP_RPM_TOLERANCE    250.0f
#define ROLLER_STARTUP_STABLE_MS         30U
#define ROLLER_STARTUP_TIMEOUT_MS      3000U

/* PIDが各VESCへ指令できるモーター電流の上限です。 */
#define ROLLER_MAX_CURRENT_A             10.0f
#define ROLLER_PID_CURRENT_RAMP_MS        300U
#define ROLLER_BRAKE_MAX_CURRENT_A       2.0f

/*
 * kimura_rollerを参考にしたVESC電流指令用のゲインです。
 * 2台の合計トルクが増えるため、実機で再調整してください。
 */
#define ROLLER_SPEED_KP                  0.020f
#define ROLLER_SPEED_KI                  0.010f
#define ROLLER_SPEED_KD                  0.000002f

/* ボタン解放後は0 RPMを目標に弱いPID制動をかけます。 */
#define ROLLER_BRAKE_KP                  0.0005f
#define ROLLER_BRAKE_KI                  0.0f
#define ROLLER_BRAKE_KD                  0.0f
#define ROLLER_STOP_RPM                100

/* 10msごとにVESCへCAN指令を送ります。 */
#define CONTROL_PERIOD_MS              10U

/* ボタンのチャタリング対策です。30ms同じ状態が続いたら確定します。 */
#define BUTTON_DEBOUNCE_MS             30U

/* 制動が長時間継続しないための安全上限です。 */
#define BRAKE_TIMEOUT_MS              1000U

/* PC3は内部プルアップ入力です。ボタンでGNDへ接続すると押下になります。 */
#define ROLLER_BUTTON_ACTIVE_STATE     GPIO_PIN_RESET

/*
 * AMT102-VのDIPスイッチ設定に合わせて変更してください。TIM8の
 * Encoder Mode TI12はA/B両相の両エッジを数えるためX4になります。
 */
#define ROLLER_ENCODER_PPR            2048U
#define ROLLER_ENCODER_COUNTS_PER_REV (ROLLER_ENCODER_PPR * 4U)
#define ROLLER_ENCODER_UPDATE_MS       10U

/* TIM8の10ms差分からPID用RPMを計算します。シリアル表示は100ms周期です。 */
#define SERIAL_MONITOR_BAUDRATE      115200U
#define SERIAL_REPORT_PERIOD_MS         100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
static VescCore g_vesc;
static CanCube g_can_cube;
static CanCubeOps g_can_ops;
static CanStm32Context g_can_context;

static RollerEncoderData g_roller_encoder;
static VescStatus5Data g_vesc_motor_1_status5;
static VescStatus5Data g_vesc_motor_2_status5;

static const PidParameter g_roller_speed_pid_parameter = {
  .gain = {
    .kp = ROLLER_SPEED_KP,
    .ki = ROLLER_SPEED_KI,
    .kd = ROLLER_SPEED_KD,
  },
  .min = -ROLLER_MAX_CURRENT_A,
  .max = ROLLER_MAX_CURRENT_A,
};

static const PidParameter g_roller_brake_pid_parameter = {
  .gain = {
    .kp = ROLLER_BRAKE_KP,
    .ki = ROLLER_BRAKE_KI,
    .kd = ROLLER_BRAKE_KD,
  },
  .min = -ROLLER_BRAKE_MAX_CURRENT_A,
  .max = ROLLER_BRAKE_MAX_CURRENT_A,
};

static PidController g_roller_speed_pid;
static PidController g_roller_brake_pid;

static VescAppState g_app_state = VESC_APP_STOPPED;
static uint32_t g_last_control_tick = 0U;
static uint32_t g_last_button_change_tick = 0U;
static uint32_t g_brake_start_tick = 0U;
static uint32_t g_startup_start_tick = 0U;
static uint32_t g_startup_duty_ramp_start_tick = 0U;
static uint32_t g_startup_rpm_in_range_start_tick = 0U;
static uint32_t g_pid_current_ramp_start_tick = 0U;
static uint32_t g_last_led_tick = 0U;
static uint32_t g_last_serial_report_tick = 0U;

/* 起動ランプ、速度PID、制動PIDが現在VESCへ送っている電流指令です。 */
static float g_pid_output_current_a = 0.0f;
static float g_pid_current_limit_a = 0.0f;
/* duty起動中に2台のVESCへ送っている共通指令です。 */
static float g_startup_duty_command = 0.0f;
static bool g_startup_rpm_in_range = false;
static bool g_pid_current_ramp_active = false;

/* CAN送信診断値です。3回連続失敗した場合だけ安全停止します。 */
static uint8_t g_can_tx_consecutive_failures = 0U;
static uint8_t g_can_tx_last_failed_vesc_id = 0U;
static uint8_t g_can_tx_last_failure_reason = 0U;
static uint32_t g_can_tx_last_hal_error = 0U;
static uint32_t g_can_tx_last_esr = 0U;

static GPIO_PinState g_button_raw_previous = GPIO_PIN_RESET;
static GPIO_PinState g_button_stable_state = GPIO_PIN_RESET;

static bool g_command_inhibit_until_release = false;
static bool g_tx_fault_latched = false;
static bool g_rx_overflow_latched = false;
static bool g_encoder_feedback_fault_latched = false;
static bool g_startup_timeout_fault_latched = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void VescApp_Init(void);
static void VescApp_Update(void);
static GPIO_PinState VescApp_ReadDebouncedButton(uint32_t now);
static void VescApp_PollCanRx(uint32_t now);
static void VescApp_ParseStatus5(const CanMessage* msg, uint32_t now);
static bool VescApp_IsStatus5Fresh(const VescStatus5Data* status,
                                   uint32_t now);
static float VescApp_GetAverageInputVoltage(uint32_t now);
static float VescApp_GetCorrectedInputVoltage(uint32_t now);
static void RollerEncoder_Init(void);
static void RollerEncoder_Update(uint32_t now);
static void SerialMonitor_Init(void);
static void SerialMonitor_Write(const char* text);
static void SerialMonitor_Update(uint32_t now);
static bool VescApp_SendCurrent(float current_a);
static bool VescApp_SendCurrentToMotor(float current_a, uint8_t vesc_can_id);
static bool VescApp_SendDuty(float duty);
static bool VescApp_SendDutyToMotor(float duty, uint8_t vesc_can_id);
static bool VescApp_SendZeroCurrent(void);
static bool VescApp_SendGeneratedVescFrame(uint8_t vesc_can_id);
static bool VescApp_RecordCommandTxResult(bool success);
static bool VescApp_IsRollerStopped(void);
static bool VescApp_HasReachedDutySwitchRpm(void);
static bool VescApp_IsStartupRpmInRange(void);
static float VescApp_GetStartupTargetDuty(uint32_t now);
static void VescApp_LatchTxFault(void);
static void VescApp_LatchEncoderFault(void);
static void VescApp_LatchStartupTimeoutFault(void);
static void VescApp_UpdateStatusLed(uint32_t now);

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
  MX_CAN1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  SerialMonitor_Init();
  VescApp_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    VescApp_Update();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  /* PC6=TIM8_CH1(A相)、PC7=TIM8_CH2(B相) */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  TIM8->CR1 = 0U;
  TIM8->CR2 = 0U;
  TIM8->SMCR = 0U;
  TIM8->DIER = 0U;
  TIM8->CCER = 0U;
  TIM8->PSC = 0U;
  TIM8->ARR = 0xFFFFU;

  /* TI1/TI2を直接入力し、デジタルフィルタ4を両相へ設定します。 */
  TIM8->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                (4U << TIM_CCMR1_IC1F_Pos) |
                (4U << TIM_CCMR1_IC2F_Pos);

  /* Encoder Mode 3: A/B両相の両エッジを数えるX4デコードです。 */
  TIM8->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM8->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM8->CNT = 0U;
  TIM8->EGR = TIM_EGR_UG;
  TIM8->CR1 = TIM_CR1_CEN;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /* PC3外付けローラーボタン: ボタンを押すとGNDへ接続されます。 */
  GPIO_InitStruct.Pin = ROLLER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROLLER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void VescApp_Init(void)
{
  /*
   * おむらいすライブラリのVESC用データ構造を初期化します。
   * VESCのCANフレーム生成と、VESCから返ってくる状態データの解析に使います。
  */
  om_vesc_core_init_in_place(&g_vesc);
  om_vesc_core_set_max_current(&g_vesc, ROLLER_MAX_CURRENT_A);
  om_vesc_core_set_max_duty(&g_vesc, ROLLER_STARTUP_DUTY_MAX_ABS);

  /* kimura_rollerと同様に、回転用PIDと停止用PIDを別々に持ちます。 */
  g_roller_speed_pid = om_pid_init(g_roller_speed_pid_parameter);
  g_roller_brake_pid = om_pid_init(g_roller_brake_pid_parameter);

  /*
   * おむらいすライブラリのCAN-Cube/STM32アダプタをCAN1に接続します。
   * 受信はこの仕組みでキューに入れ、メインループ側で安全に取り出します。
   */
  can_stm32_make_ops(&g_can_ops);
  can_stm32_context_init(&g_can_context, &g_can_cube, &hcan1,
                         CAN_STM32_KIND_CAN, CAN_RX_FIFO0);
  can_cube_init(&g_can_cube, &g_can_context, &g_can_ops);

  if (!can_stm32_register(&g_can_context)) {
    Error_Handler();
  }

  /*
   * CAN受信割り込みを有効にし、CAN1を開始します。
   * フィルタは全受信にして、VESCのSTATUSだけを後でソフト側で選別します。
   */
  can_cube_start_read(&g_can_cube);

  /* PC6=A相、PC7=B相をTIM8 Encoder Modeで直接計数します。 */
  RollerEncoder_Init();

  g_button_raw_previous = HAL_GPIO_ReadPin(ROLLER_BUTTON_GPIO_Port,
                                           ROLLER_BUTTON_Pin);
  g_button_stable_state = g_button_raw_previous;
  g_last_button_change_tick = HAL_GetTick();
}

static void VescApp_Update(void)
{
  const uint32_t now = HAL_GetTick();

  VescApp_PollCanRx(now);
  RollerEncoder_Update(now);
  SerialMonitor_Update(now);

  const GPIO_PinState button_state = VescApp_ReadDebouncedButton(now);
  const bool button_pressed = (button_state == ROLLER_BUTTON_ACTIVE_STATE);

  /*
   * 送信失敗などが起きた後は、いったんボタンを離すまで再始動しません。
   * 押しっぱなしで突然再始動するのを避けるための安全処理です。
   */
  if (!button_pressed) {
    g_command_inhibit_until_release = false;
    g_tx_fault_latched = false;
    g_encoder_feedback_fault_latched = false;
    g_startup_timeout_fault_latched = false;
  }

  /* 回転中にエンコーダ情報が途絶えた場合は、即座に0Aへ戻します。 */
  if (button_pressed && !g_roller_encoder.fresh) {
    VescApp_LatchEncoderFault();
  }

  if (button_pressed && !g_command_inhibit_until_release &&
      !g_tx_fault_latched && !g_encoder_feedback_fault_latched) {
    /* PC3ボタンを押した瞬間に150msの電流ランプを開始します。 */
    if (g_app_state != VESC_APP_STARTING_CURRENT &&
        g_app_state != VESC_APP_STARTING_DUTY &&
        g_app_state != VESC_APP_DRIVING) {
      om_pid_reset(&g_roller_speed_pid);
      om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                       ROLLER_MAX_CURRENT_A);
      om_pid_reset(&g_roller_brake_pid);
      g_pid_output_current_a = 0.0f;
      g_pid_current_limit_a = 0.0f;
      g_pid_current_ramp_active = false;
      g_startup_duty_command = 0.0f;
      g_startup_start_tick = now;
      g_startup_duty_ramp_start_tick = 0U;
      g_startup_rpm_in_range_start_tick = 0U;
      g_startup_rpm_in_range = false;
      g_can_tx_consecutive_failures = 0U;
      g_roller_encoder.rpm_updated = false;
      g_app_state = VESC_APP_STARTING_CURRENT;
    }
  } else if (g_app_state == VESC_APP_STARTING_CURRENT ||
             g_app_state == VESC_APP_STARTING_DUTY ||
             g_app_state == VESC_APP_DRIVING) {
    /*
     * ボタンを離した瞬間に速度PIDをリセットし、0 RPMを目標とする
     * 制動PIDへ切り替えます。
     */
    om_pid_reset(&g_roller_speed_pid);
    om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                     ROLLER_MAX_CURRENT_A);
    om_pid_reset(&g_roller_brake_pid);
    g_app_state = VescApp_IsRollerStopped() ? VESC_APP_STOPPED
                                            : VESC_APP_BRAKING;
    g_brake_start_tick = now;
    g_pid_output_current_a = 0.0f;
    g_pid_current_limit_a = 0.0f;
    g_pid_current_ramp_active = false;
    g_startup_duty_command = 0.0f;
    g_startup_rpm_in_range = false;
  }

  if ((uint32_t)(now - g_last_control_tick) >= CONTROL_PERIOD_MS) {
    g_last_control_tick = now;

    switch (g_app_state) {
      case VESC_APP_STARTING_CURRENT: {
        if (!g_roller_encoder.fresh) {
          VescApp_LatchEncoderFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        const uint32_t startup_elapsed_ms = now - g_startup_start_tick;
        if (startup_elapsed_ms >= ROLLER_STARTUP_TIMEOUT_MS) {
          VescApp_LatchStartupTimeoutFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        /* 0から2.0A/台まで150msで直線的に増加させます。 */
        float current_ramp_ratio =
            (float)startup_elapsed_ms /
            (float)ROLLER_STARTUP_CURRENT_RAMP_MS;
        if (current_ramp_ratio > 1.0f) {
          current_ramp_ratio = 1.0f;
        }
        const float current_direction =
            (ROLLER_TARGET_RPM < 0.0f) ? -1.0f : 1.0f;
        const float startup_current_a =
            current_direction * ROLLER_STARTUP_CURRENT_A *
            current_ramp_ratio;

        if (!VescApp_SendCurrent(startup_current_a)) {
          VescApp_LatchTxFault();
          break;
        }

        /*
         * 電流ランプ完了後、指令方向へ300RPM以上になったらdutyランプへ
         * 切り替えます。逆方向の回転では切り替えません。
         */
        if (startup_elapsed_ms >= ROLLER_STARTUP_CURRENT_RAMP_MS &&
            g_roller_encoder.rpm_updated) {
          if (VescApp_HasReachedDutySwitchRpm()) {
            g_startup_duty_ramp_start_tick = now;
            g_startup_duty_command = 0.0f;
            g_pid_output_current_a = 0.0f;
            g_app_state = VESC_APP_STARTING_DUTY;
          }
          g_roller_encoder.rpm_updated = false;
        }
        break;
      }

      case VESC_APP_STARTING_DUTY: {
        if (!g_roller_encoder.fresh) {
          VescApp_LatchEncoderFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        const uint32_t startup_elapsed_ms = now - g_startup_start_tick;
        if (startup_elapsed_ms >= ROLLER_STARTUP_TIMEOUT_MS) {
          VescApp_LatchStartupTimeoutFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        const uint32_t duty_elapsed_ms =
            now - g_startup_duty_ramp_start_tick;
        float duty_ramp_ratio =
            (float)duty_elapsed_ms /
            (float)ROLLER_STARTUP_DUTY_RAMP_MS;
        if (duty_ramp_ratio > 1.0f) {
          duty_ramp_ratio = 1.0f;
        }

        const float duty_direction =
            (ROLLER_TARGET_RPM < 0.0f) ? -1.0f : 1.0f;
        const float duty_begin =
            duty_direction * ROLLER_STARTUP_DUTY_BEGIN_ABS;
        const float duty_target = VescApp_GetStartupTargetDuty(now);
        g_startup_duty_command =
            duty_begin + (duty_target - duty_begin) * duty_ramp_ratio;
        g_pid_output_current_a = 0.0f;

        if (!VescApp_SendDuty(g_startup_duty_command)) {
          VescApp_LatchTxFault();
          break;
        }

        /*
         * 目標±250RPMを30ms維持したらPIDへ移ります。ランプ途中でも
         * 目標に達した場合は、過回転を避けるためPIDへ切り替えます。
         * 新しいエンコーダRPMが来た時だけ判定し、古い値では遷移しません。
         */
        if (g_roller_encoder.rpm_updated) {
          if (VescApp_IsStartupRpmInRange()) {
            if (!g_startup_rpm_in_range) {
              g_startup_rpm_in_range = true;
              g_startup_rpm_in_range_start_tick = now;
            } else if ((uint32_t)(now - g_startup_rpm_in_range_start_tick) >=
                       ROLLER_STARTUP_STABLE_MS) {
              om_pid_reset(&g_roller_speed_pid);
              om_pid_set_limit(&g_roller_speed_pid, 0.0f, 0.0f);
              g_pid_output_current_a = 0.0f;
              g_pid_current_limit_a = 0.0f;
              g_pid_current_ramp_start_tick = now;
              g_pid_current_ramp_active = true;
              g_startup_duty_command = 0.0f;
              g_startup_rpm_in_range = false;
              g_app_state = VESC_APP_DRIVING;
            }
          } else {
            g_startup_rpm_in_range = false;
          }
          g_roller_encoder.rpm_updated = false;
        }
        break;
      }

      case VESC_APP_DRIVING:
        if (!g_roller_encoder.fresh) {
          VescApp_LatchEncoderFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        /*
         * dutyからPIDへ切り替えた直後は、PID電流上限を0Aから徐々に
         * 広げます。切替時の回転数誤差による急な電流ステップを防ぎます。
         */
        if (g_pid_current_ramp_active) {
          const uint32_t pid_ramp_elapsed_ms =
              now - g_pid_current_ramp_start_tick;
          float pid_ramp_ratio =
              (float)pid_ramp_elapsed_ms /
              (float)ROLLER_PID_CURRENT_RAMP_MS;
          if (pid_ramp_ratio >= 1.0f) {
            pid_ramp_ratio = 1.0f;
            g_pid_current_ramp_active = false;
          }
          g_pid_current_limit_a =
              ROLLER_MAX_CURRENT_A * pid_ramp_ratio;
          om_pid_set_limit(&g_roller_speed_pid,
                           -g_pid_current_limit_a,
                           g_pid_current_limit_a);
        } else {
          g_pid_current_limit_a = ROLLER_MAX_CURRENT_A;
          om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                           ROLLER_MAX_CURRENT_A);
        }

        /* 新しいRPMが得られたときだけPIDを更新し、間は前回値を再送します。 */
        if (g_roller_encoder.rpm_updated) {
          g_pid_output_current_a = om_pid_calc(
              &g_roller_speed_pid, ROLLER_TARGET_RPM,
              (float)g_roller_encoder.rpm,
              g_roller_encoder.rpm_dt_sec);
          g_roller_encoder.rpm_updated = false;
        }

        if (!VescApp_SendCurrent(g_pid_output_current_a)) {
          VescApp_LatchTxFault();
        }
        break;

      case VESC_APP_BRAKING:
        g_startup_duty_command = 0.0f;
        if (!g_roller_encoder.fresh || VescApp_IsRollerStopped() ||
            (uint32_t)(now - g_brake_start_tick) >= BRAKE_TIMEOUT_MS) {
          om_pid_reset(&g_roller_brake_pid);
          g_pid_output_current_a = 0.0f;
          (void)VescApp_SendZeroCurrent();
          g_app_state = VESC_APP_STOPPED;
          break;
        }

        if (g_roller_encoder.rpm_updated) {
          g_pid_output_current_a = om_pid_calc(
              &g_roller_brake_pid, 0.0f,
              (float)g_roller_encoder.rpm,
              g_roller_encoder.rpm_dt_sec);
          g_roller_encoder.rpm_updated = false;
        }

        if (!VescApp_SendCurrent(g_pid_output_current_a)) {
          VescApp_LatchTxFault();
        }
        break;

      case VESC_APP_STOPPED:
      default:
        g_pid_output_current_a = 0.0f;
        g_startup_duty_command = 0.0f;
        g_startup_rpm_in_range = false;
        g_roller_encoder.rpm_updated = false;
        (void)VescApp_SendZeroCurrent();
        break;
    }
  }

  VescApp_UpdateStatusLed(now);
}

static GPIO_PinState VescApp_ReadDebouncedButton(uint32_t now)
{
  /*
   * ボタンは機械接点なので、押した瞬間と離した瞬間に細かくON/OFFが揺れます。
   * その揺れを「チャタリング」と呼びます。
   * ここでは30ms同じ状態が続いたら、本当に押された/離されたと判断します。
   */
  const GPIO_PinState raw = HAL_GPIO_ReadPin(ROLLER_BUTTON_GPIO_Port,
                                             ROLLER_BUTTON_Pin);

  if (raw != g_button_raw_previous) {
    g_button_raw_previous = raw;
    g_last_button_change_tick = now;
  }

  if ((uint32_t)(now - g_last_button_change_tick) >= BUTTON_DEBOUNCE_MS) {
    g_button_stable_state = raw;
  }

  return g_button_stable_state;
}

static void VescApp_PollCanRx(uint32_t now)
{
  CanMessage msg;

  /*
   * 割り込みで受信キューに入ったCANメッセージを取り出します。
   * 標準STATUSはおむらいすライブラリ、電源電圧を含むSTATUS_5は
   * このアプリで解析します。
   */
  while (can_cube_poll(&g_can_cube, &msg)) {
    VescApp_ParseStatus5(&msg, now);
    (void)om_vesc_core_parse(&g_vesc, msg.id, msg.data);
  }

  if (can_cube_get_rx_overflow_count(&g_can_cube) != 0U) {
    g_rx_overflow_latched = true;
  }
}

static void VescApp_ParseStatus5(const CanMessage* msg, uint32_t now)
{
  if (msg == NULL || msg->len < 6U) {
    return;
  }

  const uint32_t packet_id = (msg->id >> 8) & 0xFFU;
  if (packet_id != VESC_CAN_PACKET_STATUS_5_ID) {
    return;
  }

  const uint8_t controller_id = (uint8_t)(msg->id & 0xFFU);
  VescStatus5Data* status = NULL;
  if (controller_id == (uint8_t)VESC_MOTOR_1_CAN_ID) {
    status = &g_vesc_motor_1_status5;
  } else if (controller_id == (uint8_t)VESC_MOTOR_2_CAN_ID) {
    status = &g_vesc_motor_2_status5;
  } else {
    return;
  }

  /* STATUS_5のB4-B5はビッグエンディアン、0.1 V単位です。 */
  status->input_voltage_x10 =
      (int16_t)(((uint16_t)msg->data[4] << 8) | (uint16_t)msg->data[5]);
  status->last_update_tick = now;
  status->valid = true;
}

static bool VescApp_IsStatus5Fresh(const VescStatus5Data* status,
                                   uint32_t now)
{
  return status != NULL && status->valid &&
         (uint32_t)(now - status->last_update_tick) <=
             VESC_STATUS_5_TIMEOUT_MS;
}

static float VescApp_GetAverageInputVoltage(uint32_t now)
{
  const bool motor_1_fresh =
      VescApp_IsStatus5Fresh(&g_vesc_motor_1_status5, now) &&
      g_vesc_motor_1_status5.input_voltage_x10 > 0;
  const bool motor_2_fresh =
      VescApp_IsStatus5Fresh(&g_vesc_motor_2_status5, now) &&
      g_vesc_motor_2_status5.input_voltage_x10 > 0;

  if (motor_1_fresh && motor_2_fresh) {
    return ((float)g_vesc_motor_1_status5.input_voltage_x10 +
            (float)g_vesc_motor_2_status5.input_voltage_x10) /
           20.0f;
  }
  if (motor_1_fresh) {
    return (float)g_vesc_motor_1_status5.input_voltage_x10 / 10.0f;
  }
  if (motor_2_fresh) {
    return (float)g_vesc_motor_2_status5.input_voltage_x10 / 10.0f;
  }

  /* VESCのSTATUS_5をまだ受信していない起動直後だけ公称値を使います。 */
  return ROLLER_NOMINAL_BATTERY_V;
}

static float VescApp_GetCorrectedInputVoltage(uint32_t now)
{
  const float average_voltage = VescApp_GetAverageInputVoltage(now);
  return average_voltage * ROLLER_BATTERY_VOLTAGE_GAIN +
         ROLLER_BATTERY_VOLTAGE_OFFSET_V;
}

static void RollerEncoder_Init(void)
{
  TIM8->CNT = 0U;
  SET_BIT(TIM8->CR1, TIM_CR1_CEN);

  g_roller_encoder.previous_count = (uint16_t)TIM8->CNT;
  g_roller_encoder.continuous_count = 0;
  g_roller_encoder.last_update_tick = HAL_GetTick();
  g_roller_encoder.rpm = 0;
  g_roller_encoder.rpm_dt_sec = 0.0f;
  g_roller_encoder.rpm_updated = false;
  g_roller_encoder.valid = true;
  g_roller_encoder.fresh = true;
}

static void RollerEncoder_Update(uint32_t now)
{
  const uint32_t elapsed_ms = now - g_roller_encoder.last_update_tick;
  if (!g_roller_encoder.valid || elapsed_ms < ROLLER_ENCODER_UPDATE_MS) {
    return;
  }

  const uint16_t count = (uint16_t)TIM8->CNT;

  /* TIM8は16bitなので、符号付き16bit差分でオーバーフローを吸収します。 */
  const int16_t delta_count =
      (int16_t)(count - g_roller_encoder.previous_count);
  const int64_t numerator = (int64_t)delta_count * 60000LL;
  const int64_t denominator =
      (int64_t)elapsed_ms * (int64_t)ROLLER_ENCODER_COUNTS_PER_REV;

  if (denominator != 0LL) {
    g_roller_encoder.rpm = (int32_t)(numerator / denominator);
    g_roller_encoder.rpm_dt_sec = (float)elapsed_ms / 1000.0f;
    g_roller_encoder.rpm_updated = true;
    g_roller_encoder.fresh = true;
  }

  g_roller_encoder.continuous_count += delta_count;
  g_roller_encoder.previous_count = count;
  g_roller_encoder.last_update_tick = now;
}

static void SerialMonitor_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  /* NUCLEO-F446REのST-LINK仮想COMポート: PA2=TX、PA3=RX */
  gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &gpio);

  CLEAR_BIT(USART2->CR1, USART_CR1_UE);
  USART2->CR1 = 0U;
  USART2->CR2 = 0U;
  USART2->CR3 = 0U;
  USART2->BRR =
      (HAL_RCC_GetPCLK1Freq() + (SERIAL_MONITOR_BAUDRATE / 2U)) /
      SERIAL_MONITOR_BAUDRATE;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

  SerialMonitor_Write("Roller PID Teleplot monitor ready\r\n");
}

static void SerialMonitor_Write(const char* text)
{
  if (text == NULL) {
    return;
  }

  while (*text != '\0') {
    const uint32_t start = HAL_GetTick();
    while ((USART2->SR & USART_SR_TXE) == 0U) {
      if ((uint32_t)(HAL_GetTick() - start) > 10U) {
        return;
      }
    }
    USART2->DR = (uint8_t)*text;
    text++;
  }
}

static void SerialMonitor_Update(uint32_t now)
{
  char line[80];

  if ((uint32_t)(now - g_last_serial_report_tick) <
      SERIAL_REPORT_PERIOD_MS) {
    return;
  }
  g_last_serial_report_tick = now;

  if (!g_roller_encoder.fresh) {
    SerialMonitor_Write("RPM=no-data\r\n");
  } else {
    /* Teleplot形式: TIM8で直接計数したAMT102-Vのローラー回転数です。 */
    (void)snprintf(line, sizeof(line), ">ROLLER_RPM:%ld\r\n",
                   (long)g_roller_encoder.rpm);
    SerialMonitor_Write(line);
  }

  /*
   * 2台のVESC（ID 105/112）へ共通で送る電流指令です。
   * 起動電流ランプまたはPID出力で、実測電流ではありません。単位はmAです。
   */
  (void)snprintf(line, sizeof(line),
                 ">ROLLER_CURRENT_mA:%ld\r\n",
                 (long)(g_pid_output_current_a * 1000.0f));
  SerialMonitor_Write(line);

  (void)snprintf(line, sizeof(line),
                 ">ROLLER_PID_CURRENT_LIMIT_mA:%ld\r\n",
                 (long)(g_pid_current_limit_a * 1000.0f));
  SerialMonitor_Write(line);

  /* 起動中のdutyランプです。0.78はTeleplot上で7800と表示されます。 */
  (void)snprintf(line, sizeof(line),
                 ">ROLLER_DUTY_x10000:%ld\r\n",
                 (long)(g_startup_duty_command * 10000.0f));
  SerialMonitor_Write(line);

  /*
   * VESC STATUS_5の入力電圧です。2台を個別表示し、受信が1秒以上
   * 途絶えたVESCの古い値はTeleplotへ出力しません。
   */
  if (VescApp_IsStatus5Fresh(&g_vesc_motor_1_status5, now)) {
    (void)snprintf(line, sizeof(line),
                   ">VESC_105_INPUT_VOLTAGE_mV:%ld\r\n",
                   (long)g_vesc_motor_1_status5.input_voltage_x10 * 100L);
    SerialMonitor_Write(line);
  }

  if (VescApp_IsStatus5Fresh(&g_vesc_motor_2_status5, now)) {
    (void)snprintf(line, sizeof(line),
                   ">VESC_112_INPUT_VOLTAGE_mV:%ld\r\n",
                   (long)g_vesc_motor_2_status5.input_voltage_x10 * 100L);
    SerialMonitor_Write(line);
  }

  const float average_input_voltage =
      VescApp_GetAverageInputVoltage(now);
  (void)snprintf(line, sizeof(line),
                 ">VESC_AVERAGE_INPUT_VOLTAGE_mV:%ld\r\n",
                 (long)(average_input_voltage * 1000.0f));
  SerialMonitor_Write(line);

  const float corrected_input_voltage =
      VescApp_GetCorrectedInputVoltage(now);
  (void)snprintf(line, sizeof(line),
                 ">VESC_CORRECTED_INPUT_VOLTAGE_mV:%ld\r\n",
                 (long)(corrected_input_voltage * 1000.0f));
  SerialMonitor_Write(line);

  (void)snprintf(line, sizeof(line),
                 ">ROLLER_TARGET_DUTY_x10000:%ld\r\n",
                 (long)(VescApp_GetStartupTargetDuty(now) * 10000.0f));
  SerialMonitor_Write(line);
}

static bool VescApp_SendCurrent(float current_a)
{
  if (current_a > ROLLER_MAX_CURRENT_A) {
    current_a = ROLLER_MAX_CURRENT_A;
  } else if (current_a < -ROLLER_MAX_CURRENT_A) {
    current_a = -ROLLER_MAX_CURRENT_A;
  }

  g_pid_output_current_a = current_a;

  /*
   * 1つのエンコーダとPID出力を共有し、2台のVESCへ同符号・
   * 同じ大きさの電流を送ります。片方の送信が失敗しても、もう
   * 片方の送信は試行し、両方が成功したときだけtrueにします。
   */
  const bool motor_1_sent =
      VescApp_SendCurrentToMotor(current_a, (uint8_t)VESC_MOTOR_1_CAN_ID);
  const bool motor_2_sent =
      VescApp_SendCurrentToMotor(current_a, (uint8_t)VESC_MOTOR_2_CAN_ID);

  return VescApp_RecordCommandTxResult(motor_1_sent && motor_2_sent);
}

static bool VescApp_SendCurrentToMotor(float current_a, uint8_t vesc_can_id)
{
  om_vesc_core_set_current(&g_vesc, current_a, vesc_can_id);
  return VescApp_SendGeneratedVescFrame(vesc_can_id);
}

static bool VescApp_SendDuty(float duty)
{
  if (duty > ROLLER_STARTUP_DUTY_MAX_ABS) {
    duty = ROLLER_STARTUP_DUTY_MAX_ABS;
  } else if (duty < -ROLLER_STARTUP_DUTY_MAX_ABS) {
    duty = -ROLLER_STARTUP_DUTY_MAX_ABS;
  }

  g_startup_duty_command = duty;

  /* 起動ランプ中も2台のVESCへ同符号・同じdutyを送ります。 */
  const bool motor_1_sent =
      VescApp_SendDutyToMotor(duty, (uint8_t)VESC_MOTOR_1_CAN_ID);
  const bool motor_2_sent =
      VescApp_SendDutyToMotor(duty, (uint8_t)VESC_MOTOR_2_CAN_ID);

  return VescApp_RecordCommandTxResult(motor_1_sent && motor_2_sent);
}

static bool VescApp_SendDutyToMotor(float duty, uint8_t vesc_can_id)
{
  om_vesc_core_set_duty(&g_vesc, duty, vesc_can_id);
  return VescApp_SendGeneratedVescFrame(vesc_can_id);
}

static bool VescApp_SendZeroCurrent(void)
{
  /*
   * 停止中の指令です。
   * VESC側の通信タイムアウトだけに頼らず、明示的に0Aを送ります。
   */
  return VescApp_SendCurrent(0.0f);
}

static bool VescApp_SendGeneratedVescFrame(uint8_t vesc_can_id)
{
  uint32_t can_id = 0U;
  uint8_t data[8] = {0};
  uint8_t len = 0U;
  CAN_TxHeaderTypeDef tx_header;
  uint32_t mailbox = 0U;

  om_vesc_core_get_output(&g_vesc, &can_id, data, &len);

  /*
   * 注意:
   * VESCのCANコマンドは「拡張ID」で送る必要があります。
   * CAN ID 105/112で電流指令を作るとIDは0x169/0x170になり、
   * 数値だけ見ると標準ID範囲内です。
   * そのため、おむらいすライブラリでフレームを作ったあと、HALで明示的に拡張ID送信します。
   */
  memset(&tx_header, 0, sizeof(tx_header));
  tx_header.ExtId = can_id;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = (len > 8U) ? 8U : len;
  tx_header.TransmitGlobalTime = DISABLE;

  const uint32_t retry_start_tick = HAL_GetTick();
  uint8_t last_failure_reason = VESC_CAN_TX_FAILURE_MAILBOX;

  do {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0U) {
      if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox) == HAL_OK) {
        return true;
      }
      last_failure_reason = VESC_CAN_TX_FAILURE_HAL;
    }
  } while ((uint32_t)(HAL_GetTick() - retry_start_tick) <
           VESC_CAN_TX_RETRY_TIMEOUT_MS);

  g_can_tx_last_failed_vesc_id = vesc_can_id;
  g_can_tx_last_failure_reason = last_failure_reason;
  g_can_tx_last_hal_error = HAL_CAN_GetError(&hcan1);
  g_can_tx_last_esr = hcan1.Instance->ESR;
  return false;
}

static bool VescApp_RecordCommandTxResult(bool success)
{
  if (success) {
    g_can_tx_consecutive_failures = 0U;
    return true;
  }

  if (g_can_tx_consecutive_failures < UINT8_MAX) {
    g_can_tx_consecutive_failures++;
  }

  /* 1～2回目は次の10 ms制御周期で再試行し、3回目で停止します。 */
  return g_can_tx_consecutive_failures < VESC_CAN_TX_FAILURE_LIMIT;
}

static bool VescApp_IsRollerStopped(void)
{
  if (!g_roller_encoder.fresh) {
    return false;
  }

  const int32_t rpm = g_roller_encoder.rpm;
  return (rpm > -ROLLER_STOP_RPM) && (rpm < ROLLER_STOP_RPM);
}

static bool VescApp_HasReachedDutySwitchRpm(void)
{
  const float rpm = (float)g_roller_encoder.rpm;
  if (ROLLER_TARGET_RPM < 0.0f) {
    return rpm <= -ROLLER_STARTUP_DUTY_SWITCH_RPM;
  }
  if (ROLLER_TARGET_RPM > 0.0f) {
    return rpm >= ROLLER_STARTUP_DUTY_SWITCH_RPM;
  }
  return false;
}

static bool VescApp_IsStartupRpmInRange(void)
{
  float rpm_error =
      ROLLER_TARGET_RPM - (float)g_roller_encoder.rpm;
  if (rpm_error < 0.0f) {
    rpm_error = -rpm_error;
  }
  return rpm_error <= ROLLER_STARTUP_RPM_TOLERANCE;
}

static float VescApp_GetStartupTargetDuty(uint32_t now)
{
  float battery_voltage = VescApp_GetCorrectedInputVoltage(now);
  if (battery_voltage < ROLLER_MIN_CALC_BATTERY_V) {
    battery_voltage = ROLLER_NOMINAL_BATTERY_V;
  }

  const float roller_rpm_at_full_duty =
      ROLLER_MOTOR_KV_RPM_PER_V * battery_voltage *
      (ROLLER_MOTOR_GEAR_TEETH / ROLLER_GEAR_TEETH);
  if (roller_rpm_at_full_duty <= 0.0f) {
    return 0.0f;
  }

  float duty =
      (ROLLER_TARGET_RPM / roller_rpm_at_full_duty) *
      ROLLER_DUTY_LOAD_COMPENSATION;
  if (duty > ROLLER_STARTUP_DUTY_MAX_ABS) {
    duty = ROLLER_STARTUP_DUTY_MAX_ABS;
  } else if (duty < -ROLLER_STARTUP_DUTY_MAX_ABS) {
    duty = -ROLLER_STARTUP_DUTY_MAX_ABS;
  }
  return duty;
}

static void VescApp_LatchTxFault(void)
{
  /*
   * CAN送信に失敗した場合は停止状態に戻し、ボタンを離すまで再始動を禁止します。
   * CAN配線ミスやVESC電源OFFのまま押し続けた時の安全側処理です。
   */
  g_tx_fault_latched = true;
  g_command_inhibit_until_release = true;
  g_app_state = VESC_APP_STOPPED;
  g_pid_output_current_a = 0.0f;
  g_pid_current_limit_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_rpm_in_range = false;
  om_pid_reset(&g_roller_speed_pid);
  om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                   ROLLER_MAX_CURRENT_A);
  om_pid_reset(&g_roller_brake_pid);

  char line[112];
  const char* reason =
      (g_can_tx_last_failure_reason == VESC_CAN_TX_FAILURE_MAILBOX)
          ? "mailbox-timeout"
          : "HAL-error";
  (void)snprintf(line, sizeof(line),
                 "CAN_TX_FAULT:id=%u,reason=%s,hal=0x%08lX,esr=0x%08lX\r\n",
                 (unsigned int)g_can_tx_last_failed_vesc_id, reason,
                 (unsigned long)g_can_tx_last_hal_error,
                 (unsigned long)g_can_tx_last_esr);
  SerialMonitor_Write(line);
}

static void VescApp_LatchEncoderFault(void)
{
  g_encoder_feedback_fault_latched = true;
  g_command_inhibit_until_release = true;
  g_app_state = VESC_APP_STOPPED;
  g_pid_output_current_a = 0.0f;
  g_pid_current_limit_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_rpm_in_range = false;
  om_pid_reset(&g_roller_speed_pid);
  om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                   ROLLER_MAX_CURRENT_A);
  om_pid_reset(&g_roller_brake_pid);
}

static void VescApp_LatchStartupTimeoutFault(void)
{
  g_startup_timeout_fault_latched = true;
  g_command_inhibit_until_release = true;
  g_app_state = VESC_APP_STOPPED;
  g_pid_output_current_a = 0.0f;
  g_pid_current_limit_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_rpm_in_range = false;
  om_pid_reset(&g_roller_speed_pid);
  om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                   ROLLER_MAX_CURRENT_A);
  om_pid_reset(&g_roller_brake_pid);
  SerialMonitor_Write("STARTUP=timeout\r\n");
}

static void VescApp_UpdateStatusLed(uint32_t now)
{
  /*
   * PA5のLED表示:
   * - 停止中: 消灯
   * - 回転指令中: 点灯
   * - ブレーキ中: ゆっくり点滅
   * - CAN送信失敗/受信あふれ: 速く点滅
   */
  if (g_tx_fault_latched || g_rx_overflow_latched ||
      g_encoder_feedback_fault_latched ||
      g_startup_timeout_fault_latched) {
    if ((uint32_t)(now - g_last_led_tick) >= 100U) {
      HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
      g_last_led_tick = now;
    }
    return;
  }

  if (g_app_state == VESC_APP_STARTING_CURRENT ||
      g_app_state == VESC_APP_STARTING_DUTY ||
      g_app_state == VESC_APP_DRIVING) {
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
    return;
  }

  if (g_app_state == VESC_APP_BRAKING) {
    if ((uint32_t)(now - g_last_led_tick) >= 150U) {
      HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
      g_last_led_tick = now;
    }
    return;
  }

  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
