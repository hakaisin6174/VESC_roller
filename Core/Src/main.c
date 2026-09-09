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
#include "dji/robomas.h"
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

typedef struct {
  int32_t electrical_rpm;
  int16_t motor_current_x10;
  int16_t duty_x1000;
  float duty_filtered_current_a;
  uint16_t duty_sample_count;
  uint32_t last_update_tick;
  bool valid;
} VescStatus1Data;

typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  GPIO_PinState raw_state;
  GPIO_PinState stable_state;
  uint32_t last_change_tick;
} DebouncedInput;

typedef struct {
  uint32_t last_update_tick;
  bool valid;
} M2006FeedbackState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 2台のVESCはCAN1に接続し、同じ電流/duty指令で同期させます。 */
#define VESC_MOTOR_1_CAN_ID            105U
#define VESC_MOTOR_2_CAN_ID            112U

/* VESC CAN_PACKET_STATUS: eRPM[4] + motor current x10[2] + duty x1000[2] */
#define VESC_CAN_PACKET_STATUS_1_ID       9U
#define VESC_STATUS_1_TIMEOUT_MS        250U

/* VESC CAN_PACKET_STATUS_5: tachometer[4 byte] + input voltage x10[2 byte] */
#define VESC_CAN_PACKET_STATUS_5_ID      27U
#define VESC_STATUS_5_TIMEOUT_MS       1000U

/* 一時的なCAN送信メールボックス混雑を即時故障扱いしないための設定です。 */
#define VESC_CAN_TX_RETRY_TIMEOUT_MS       2U
#define VESC_CAN_TX_FAILURE_LIMIT          3U
#define VESC_CAN_TX_FAILURE_MAILBOX        1U
#define VESC_CAN_TX_FAILURE_HAL            2U

/*
 * CAN2の標準ID 200でdata[0]=1を受信するとPC3押下、data[0]=0を
 * 受信するとPC3解放と同じ動作をします。最後の受信からタイムアウト
 * した場合も、自動的にボタンを離した状態へ戻します。
 */
#define CAN2_REMOTE_TRIGGER_STD_ID        200U
#define CAN2_REMOTE_TRIGGER_ON_VALUE        1U
#define CAN2_REMOTE_TRIGGER_OFF_VALUE       0U
#define CAN2_REMOTE_TRIGGER_TIMEOUT_MS    100U
#define CAN2_REMOTE_TRIGGER_FILTER_BANK    14U

/*
 * kimura_rollerから移植した装填用M2006です。C610側のIDも4/5/6へ
 * 合わせてください。ID5とID6は同方向へ3:8の回転数比で回します。
 */
#define M2006_REPLACEMENT_ID                4
#define M2006_FEED_1_ID                     5
#define M2006_FEED_2_ID                     6
#define M2006_FEED_2_TARGET_RPM       10000.0f
#define M2006_FEED_1_TARGET_RPM       (M2006_FEED_2_TARGET_RPM * 3.0f / 8.0f)
#define M2006_FEED_TARGET_SIGN             (-1.0f)
#define M2006_MAX_CURRENT_COMMAND        4000.0f
#define M2006_SPEED_KP                      2.0f
#define M2006_SPEED_KI                      0.1f
#define M2006_SPEED_KD                      0.0f

/* ID4はPC3/PA1で正転、PB10で逆転します。 */
#define M2006_REPLACEMENT_FORWARD_TARGET_RPM      300.0f
#define M2006_REPLACEMENT_REVERSE_TARGET_RPM      1500.0f
#define M2006_REPLACEMENT_MAX_CURRENT    4000.0f
#define M2006_REPLACEMENT_KP                7.0f
#define M2006_REPLACEMENT_KI                2.0f
#define M2006_REPLACEMENT_KD                0.0f
#define M2006_AUTO_REVERSE_STOP_MS       3000U
/*
 * M2006側の安全停止条件です。
 * VESCと同じCAN1にC610を追加しているため、短時間のCAN混雑や
 * フィードバック遅れでは即停止しないよう少し余裕を持たせます。
 */
#define M2006_FEEDBACK_TIMEOUT_MS         1000U
#define M2006_FEEDBACK_STARTUP_GRACE_MS    500U
#define M2006_CAN_TX_FAILURE_LIMIT         100U
#define M2006_FEEDBACK_FAULT_STOP_ENABLE     0U
#define M2006_CAN_TX_FAULT_STOP_ENABLE       0U
#define M2006_LOAD_START_ROLLER_RPM       4000.0f
#define M2006_BUTTON_ACTIVE_STATE GPIO_PIN_RESET

/*
 * 射出ローラーの目標回転数です。
 * 逆方向へ回す場合は負の値へ変更してください。
 */
#define ROLLER_TARGET_RPM             4600.0f

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
#define ROLLER_STARTUP_CURRENT_A           3.0f
#define ROLLER_STARTUP_CURRENT_RAMP_MS    300U
#define ROLLER_STARTUP_DUTY_SWITCH_RPM    250.0f
#define ROLLER_STARTUP_DUTY_BEGIN_ABS       0.05f
#define ROLLER_STARTUP_DUTY_MAX_ABS        0.95f
#define ROLLER_STARTUP_DUTY_RAMP_MS       1800U
#define ROLLER_STARTUP_RPM_TOLERANCE    300.0f
#define ROLLER_STARTUP_STABLE_MS        100U
#define ROLLER_STARTUP_TIMEOUT_MS      5000U

/* PIDが各VESCへ指令できるモーター電流の上限です。 */
#define ROLLER_MAX_CURRENT_A             30.0f
#define ROLLER_PID_CURRENT_RAMP_MS        150U
#define ROLLER_BRAKE_MAX_CURRENT_A       2.0f

/*
 * PID切替時に足すフィードフォワード電流です。
 * 単位はA/台です。例えば2.9fなら、各VESCへ2.9Aをベースで指令し、
 * その上にPID補正電流を足します。符号は目標RPMから自動で決めます。
 */
#define ROLLER_FEEDFORWARD_CURRENT_A        2.0f

/* duty中の実測モーター電流をTeleplotへ出すためのフィルタです。 */
#define ROLLER_DUTY_CURRENT_FILTER_ALPHA   0.35f

/*
 * kimura_rollerを参考にしたVESC電流指令用のゲインです。
 * 2台の合計トルクが増えるため、実機で再調整してください。
 */
#define ROLLER_SPEED_KP                  0.017f
#define ROLLER_SPEED_KI                  0.0005f
#define ROLLER_SPEED_KD                  0.00005f

/* ボタン解放後は0 RPMを目標に弱いPID制動をかけます。 */
#define ROLLER_BRAKE_KP                  0.0005f
#define ROLLER_BRAKE_KI                  0.0f
#define ROLLER_BRAKE_KD                  0.0f
#define ROLLER_STOP_RPM                100

/* 10msごとにVESCへCAN指令を送ります。 */
#define CONTROL_PERIOD_MS              10U
#define CONTROL_PERIOD_SEC             ((float)CONTROL_PERIOD_MS / 1000.0f)

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
#define ROLLER_ENCODER_DIRECTION      (-1)
#define ROLLER_ENCODER_UPDATE_MS       10U
#define ROLLER_ENCODER_DIRECTION_CHECK_RPM 300
#define ROLLER_OVERSPEED_MARGIN_RPM   700

/* TIM8の10ms差分からPID用RPMを計算します。シリアル表示は100ms周期です。 */
#define SERIAL_MONITOR_BAUDRATE      115200U
#define SERIAL_REPORT_PERIOD_MS         100U
#define SERIAL_TX_BUFFER_SIZE           1024U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN PV */
static VescCore g_vesc;
static CanCube g_can_cube;
static CanCubeOps g_can_ops;
static CanStm32Context g_can_context;
static Robomas g_m2006;

static RollerEncoderData g_roller_encoder;
static VescStatus1Data g_vesc_motor_1_status1;
static VescStatus1Data g_vesc_motor_2_status1;
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

static const PidParameter g_m2006_speed_pid_parameter = {
  .gain = {
    .kp = M2006_SPEED_KP,
    .ki = M2006_SPEED_KI,
    .kd = M2006_SPEED_KD,
  },
  .min = -M2006_MAX_CURRENT_COMMAND,
  .max = M2006_MAX_CURRENT_COMMAND,
};

static const PidParameter g_m2006_replacement_pid_parameter = {
  .gain = {
    .kp = M2006_REPLACEMENT_KP,
    .ki = M2006_REPLACEMENT_KI,
    .kd = M2006_REPLACEMENT_KD,
  },
  .min = -M2006_REPLACEMENT_MAX_CURRENT,
  .max = M2006_REPLACEMENT_MAX_CURRENT,
};

static PidController g_roller_speed_pid;
static PidController g_roller_brake_pid;
static PidController g_m2006_replacement_pid;
static PidController g_m2006_feed_1_pid;
static PidController g_m2006_feed_2_pid;

static DebouncedInput g_m2006_forward_button;
static DebouncedInput g_m2006_reverse_button;
static DebouncedInput g_m2006_limit_reverse;
static DebouncedInput g_m2006_limit_forward;
static M2006FeedbackState g_m2006_feedback[3];
static bool g_m2006_common_button_pressed = false;
static bool g_m2006_load_enabled = false;
static bool g_m2006_load_pressed = false;
static bool g_m2006_forward_pressed = false;
static bool g_m2006_reverse_pressed = false;
static bool g_m2006_limit_reverse_active = false;
static bool g_m2006_limit_forward_active = false;
static bool g_m2006_load_was_pressed = false;
static bool g_m2006_auto_reverse_wait_active = false;
static bool g_m2006_auto_reverse_active = false;
static bool g_m2006_auto_reverse_finished = false;
static bool g_m2006_motion_was_active = false;
static uint32_t g_m2006_motion_start_tick = 0U;
static uint32_t g_m2006_auto_reverse_wait_start_tick = 0U;
static int8_t g_m2006_previous_replacement_direction = 0;
static int16_t g_m2006_replacement_output = 0;
static int16_t g_m2006_feed_1_output = 0;
static int16_t g_m2006_feed_2_output = 0;
static bool g_m2006_feedback_fault_active = false;
static bool g_m2006_tx_fault_active = false;
static bool g_m2006_feedback_timeout_reported = false;
static bool g_m2006_tx_fault_reported = false;
static uint8_t g_m2006_can_tx_consecutive_failures = 0U;

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
static char g_serial_tx_buffer[SERIAL_TX_BUFFER_SIZE];
static uint16_t g_serial_tx_head = 0U;
static uint16_t g_serial_tx_tail = 0U;

/* 起動ランプ、速度PID、制動PIDが現在VESCへ送っている電流指令です。 */
static float g_pid_output_current_a = 0.0f;
static float g_pid_current_limit_a = 0.0f;
static float g_pid_correction_current_a = 0.0f;
static float g_feedforward_current_a = 0.0f;
/* duty起動中に2台のVESCへ送っている共通指令です。 */
static float g_startup_duty_command = 0.0f;
static float g_startup_duty_hold_command = 0.0f;
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

static uint32_t g_can2_remote_trigger_last_rx_tick = 0U;
static bool g_can2_remote_trigger_valid = false;
static bool g_can2_remote_trigger_active = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void VescApp_Init(void);
static void VescApp_Update(void);
static GPIO_PinState VescApp_ReadDebouncedButton(uint32_t now);
static void DebouncedInput_Init(DebouncedInput* input, GPIO_TypeDef* port,
                                uint16_t pin, uint32_t now);
static bool DebouncedInput_IsActive(DebouncedInput* input, uint32_t now);
static void M2006App_Init(uint32_t now);
static void M2006App_UpdateInputs(uint32_t now, bool roller_button_pressed);
static void M2006App_Control(uint32_t now);
static bool M2006App_IsFeedbackFresh(int motor_id, uint32_t now);
static void M2006App_SetAllOutputsZero(void);
static bool M2006App_SendCommands(void);
static bool M2006App_SendStandardFrame(uint32_t can_id,
                                       const uint8_t data[8]);
static void VescApp_PollCanRx(uint32_t now);
static void VescApp_ParseStatus1(const CanMessage* msg, uint32_t now);
static void VescApp_ParseStatus5(const CanMessage* msg, uint32_t now);
static bool VescApp_IsStatus1Fresh(const VescStatus1Data* status,
                                   uint32_t now);
static bool VescApp_IsStatus5Fresh(const VescStatus5Data* status,
                                   uint32_t now);
static void VescApp_ResetDutyCurrentFilters(void);
static bool VescApp_IsDutyFeedforwardReady(uint32_t now);
static float VescApp_GetStartupFeedforwardCurrent(uint32_t now);
static float VescApp_GetTargetFeedforwardLimit(void);
static float VescApp_GetAverageInputVoltage(uint32_t now);
static float VescApp_GetCorrectedInputVoltage(uint32_t now);
static void RollerEncoder_Init(void);
static void RollerEncoder_Update(uint32_t now);
static void SerialMonitor_Init(void);
static void SerialMonitor_Write(const char* text);
static void SerialMonitor_ServiceTx(void);
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
static bool VescApp_IsEncoderDirectionWrong(void);
static bool VescApp_IsRollerOverspeed(void);
static float VescApp_GetStartupTargetDuty(uint32_t now);
static void VescApp_LatchTxFault(void);
static void VescApp_LatchEncoderFault(void);
static void VescApp_LatchStartupTimeoutFault(void);
static void VescApp_UpdateStatusLed(uint32_t now);
static void Can2RemoteTrigger_Init(void);
static void Can2RemoteTrigger_Poll(uint32_t now);
static bool Can2RemoteTrigger_IsActive(uint32_t now);

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
  MX_CAN2_Init();
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
  hcan1.Init.Prescaler = 3;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /* PC3外付けローラーボタン: ボタンを押すとGNDへ接続されます。 */
  GPIO_InitStruct.Pin = ROLLER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ROLLER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*
   * 装填スイッチとリミットスイッチは、接点をGNDへ落とすactive-low入力です。
   * PA1=ID4正転、PC3=3台連動、D6=ID4逆転、D7/D8=両端リミットです。
   */
  GPIO_InitStruct.Pin = M2006_FORWARD_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = M2006_REVERSE_BUTTON_Pin |
                        M2006_LIMIT_REVERSE_Pin |
                        M2006_LIMIT_FORWARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

  /* CAN2は標準ID 200だけを受信する遠隔共通ボタンとして使います。 */
  Can2RemoteTrigger_Init();

  /* PC6=A相、PC7=B相をTIM8 Encoder Modeで直接計数します。 */
  RollerEncoder_Init();

  g_button_raw_previous = HAL_GPIO_ReadPin(ROLLER_BUTTON_GPIO_Port,
                                           ROLLER_BUTTON_Pin);
  g_button_stable_state = g_button_raw_previous;
  g_last_button_change_tick = HAL_GetTick();

  /* C610/M2006の受信解析、速度PID、物理スイッチを初期化します。 */
  M2006App_Init(g_last_button_change_tick);
}

static void VescApp_Update(void)
{
  const uint32_t now = HAL_GetTick();

  SerialMonitor_ServiceTx();
  VescApp_PollCanRx(now);
  Can2RemoteTrigger_Poll(now);
  RollerEncoder_Update(now);
  SerialMonitor_Update(now);

  const GPIO_PinState button_state = VescApp_ReadDebouncedButton(now);
  const bool physical_button_pressed =
      (button_state == ROLLER_BUTTON_ACTIVE_STATE);
  const bool button_pressed =
      physical_button_pressed || Can2RemoteTrigger_IsActive(now);
  M2006App_UpdateInputs(now, button_pressed);

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
  /* PC3またはCAN2遠隔指令が有効になった瞬間に電流ランプを開始します。 */
    if (g_app_state != VESC_APP_STARTING_CURRENT &&
        g_app_state != VESC_APP_STARTING_DUTY &&
        g_app_state != VESC_APP_DRIVING) {
      om_pid_reset(&g_roller_speed_pid);
      om_pid_set_limit(&g_roller_speed_pid, -ROLLER_MAX_CURRENT_A,
                       ROLLER_MAX_CURRENT_A);
      om_pid_reset(&g_roller_brake_pid);
      g_pid_output_current_a = 0.0f;
      g_pid_current_limit_a = 0.0f;
      g_pid_correction_current_a = 0.0f;
      g_feedforward_current_a = 0.0f;
      g_pid_current_ramp_active = false;
      g_startup_duty_command = 0.0f;
      g_startup_duty_hold_command = 0.0f;
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
    g_pid_correction_current_a = 0.0f;
    g_feedforward_current_a = 0.0f;
    g_pid_current_ramp_active = false;
    g_startup_duty_command = 0.0f;
    g_startup_duty_hold_command = 0.0f;
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
        if (VescApp_IsEncoderDirectionWrong()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ENCODER=wrong-direction\r\n");
          (void)VescApp_SendZeroCurrent();
          break;
        }
        if (VescApp_IsRollerOverspeed()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ROLLER=overspeed\r\n");
          (void)VescApp_SendZeroCurrent();
          break;
        }

        const uint32_t startup_elapsed_ms = now - g_startup_start_tick;
        if (startup_elapsed_ms >= ROLLER_STARTUP_TIMEOUT_MS) {
          VescApp_LatchStartupTimeoutFault();
          (void)VescApp_SendZeroCurrent();
          break;
        }

        /* 0から2.0A/台まで100msで直線的に増加させます。 */
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
            g_startup_duty_hold_command = 0.0f;
            g_pid_output_current_a = 0.0f;
            VescApp_ResetDutyCurrentFilters();
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
        if (VescApp_IsEncoderDirectionWrong()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ENCODER=wrong-direction\r\n");
          (void)VescApp_SendZeroCurrent();
          break;
        }
        if (VescApp_IsRollerOverspeed()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ROLLER=overspeed\r\n");
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
        const float ramp_duty_command =
            duty_begin + (duty_target - duty_begin) * duty_ramp_ratio;
        if (g_startup_rpm_in_range) {
          g_startup_duty_command = g_startup_duty_hold_command;
        } else {
          g_startup_duty_command = ramp_duty_command;
        }
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
              g_startup_duty_hold_command = g_startup_duty_command;
              VescApp_ResetDutyCurrentFilters();
            } else if ((uint32_t)(now - g_startup_rpm_in_range_start_tick) >=
                           ROLLER_STARTUP_STABLE_MS &&
                       VescApp_IsDutyFeedforwardReady(now)) {
              om_pid_reset(&g_roller_speed_pid);
              om_pid_set_limit(&g_roller_speed_pid, 0.0f, 0.0f);
              g_feedforward_current_a =
                  VescApp_GetStartupFeedforwardCurrent(now);
              g_pid_correction_current_a = 0.0f;
              g_pid_output_current_a = g_feedforward_current_a;
              g_pid_current_limit_a = 0.0f;
              g_pid_current_ramp_start_tick = now;
              g_pid_current_ramp_active = true;
              g_startup_duty_command = 0.0f;
              g_startup_duty_hold_command = 0.0f;
              g_startup_rpm_in_range = false;
              g_app_state = VESC_APP_DRIVING;
            }
          } else {
            g_startup_rpm_in_range = false;
            g_startup_duty_hold_command = 0.0f;
            VescApp_ResetDutyCurrentFilters();
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
        if (VescApp_IsEncoderDirectionWrong()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ENCODER=wrong-direction\r\n");
          (void)VescApp_SendZeroCurrent();
          break;
        }
        if (VescApp_IsRollerOverspeed()) {
          VescApp_LatchEncoderFault();
          SerialMonitor_Write("ROLLER=overspeed\r\n");
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
          float available_pid_current_a =
              ROLLER_MAX_CURRENT_A -
              ((g_feedforward_current_a < 0.0f)
                   ? -g_feedforward_current_a
                   : g_feedforward_current_a);
          if (available_pid_current_a < 0.0f) {
            available_pid_current_a = 0.0f;
          }
          g_pid_current_limit_a =
              available_pid_current_a * pid_ramp_ratio;
          om_pid_set_limit(&g_roller_speed_pid,
                           -g_pid_current_limit_a,
                           g_pid_current_limit_a);
        } else {
          g_pid_current_limit_a =
              ROLLER_MAX_CURRENT_A -
              ((g_feedforward_current_a < 0.0f)
                   ? -g_feedforward_current_a
                   : g_feedforward_current_a);
          if (g_pid_current_limit_a < 0.0f) {
            g_pid_current_limit_a = 0.0f;
          }
          om_pid_set_limit(&g_roller_speed_pid, -g_pid_current_limit_a,
                           g_pid_current_limit_a);
        }

        /* 新しいRPMが得られたときだけPIDを更新し、間は前回値を再送します。 */
        if (g_roller_encoder.rpm_updated) {
          g_pid_correction_current_a = om_pid_calc(
              &g_roller_speed_pid, ROLLER_TARGET_RPM,
              (float)g_roller_encoder.rpm,
              g_roller_encoder.rpm_dt_sec);
          g_pid_output_current_a =
              g_feedforward_current_a + g_pid_correction_current_a;
          g_roller_encoder.rpm_updated = false;
        }

        if (!VescApp_SendCurrent(g_pid_output_current_a)) {
          VescApp_LatchTxFault();
        }
        break;

      case VESC_APP_BRAKING:
        g_startup_duty_command = 0.0f;
        g_startup_duty_hold_command = 0.0f;
        g_feedforward_current_a = 0.0f;
        g_pid_correction_current_a = 0.0f;
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
        g_feedforward_current_a = 0.0f;
        g_pid_correction_current_a = 0.0f;
        g_startup_duty_command = 0.0f;
        g_startup_duty_hold_command = 0.0f;
        g_startup_rpm_in_range = false;
        g_roller_encoder.rpm_updated = false;
        (void)VescApp_SendZeroCurrent();
        break;
    }

    /* VESCの2フレームに続けて、C610 ID1～4/5～8の標準ID指令を送ります。 */
    M2006App_Control(now);
  }

  VescApp_UpdateStatusLed(now);
  SerialMonitor_ServiceTx();
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

static void DebouncedInput_Init(DebouncedInput* input, GPIO_TypeDef* port,
                                uint16_t pin, uint32_t now)
{
  input->port = port;
  input->pin = pin;
  input->raw_state = HAL_GPIO_ReadPin(port, pin);
  input->stable_state = input->raw_state;
  input->last_change_tick = now;
}

static bool DebouncedInput_IsActive(DebouncedInput* input, uint32_t now)
{
  const GPIO_PinState raw = HAL_GPIO_ReadPin(input->port, input->pin);
  if (raw != input->raw_state) {
    input->raw_state = raw;
    input->last_change_tick = now;
  }

  if ((uint32_t)(now - input->last_change_tick) >= BUTTON_DEBOUNCE_MS) {
    input->stable_state = input->raw_state;
  }

  return input->stable_state == M2006_BUTTON_ACTIVE_STATE;
}

static void M2006App_Init(uint32_t now)
{
  g_m2006 = om_rm_init(can_cube_bus(&g_can_cube));
  om_rm_set_max_output(&g_m2006, (int16_t)M2006_MAX_CURRENT_COMMAND);
  g_m2006_replacement_pid = om_pid_init(g_m2006_replacement_pid_parameter);
  g_m2006_feed_1_pid = om_pid_init(g_m2006_speed_pid_parameter);
  g_m2006_feed_2_pid = om_pid_init(g_m2006_speed_pid_parameter);

  DebouncedInput_Init(&g_m2006_forward_button,
                      M2006_FORWARD_BUTTON_GPIO_Port,
                      M2006_FORWARD_BUTTON_Pin, now);
  DebouncedInput_Init(&g_m2006_reverse_button,
                      M2006_REVERSE_BUTTON_GPIO_Port,
                      M2006_REVERSE_BUTTON_Pin, now);
  DebouncedInput_Init(&g_m2006_limit_reverse,
                      M2006_LIMIT_REVERSE_GPIO_Port,
                      M2006_LIMIT_REVERSE_Pin, now);
  DebouncedInput_Init(&g_m2006_limit_forward,
                      M2006_LIMIT_FORWARD_GPIO_Port,
                      M2006_LIMIT_FORWARD_Pin, now);
  M2006App_SetAllOutputsZero();
}

static void M2006App_UpdateInputs(uint32_t now, bool roller_button_pressed)
{
  /*
   * PC3またはCAN2遠隔指令は射出ローラーと装填連動の共通入力です。
   * 装填M2006は、射出ローラーが十分に立ち上がってから動かします。
   */
  int32_t roller_rpm_abs = g_roller_encoder.rpm;
  if (roller_rpm_abs < 0) {
    roller_rpm_abs = -roller_rpm_abs;
  }
  g_m2006_common_button_pressed = roller_button_pressed;
  if (!roller_button_pressed) {
    g_m2006_load_enabled = false;
  } else if (g_roller_encoder.fresh &&
             (float)roller_rpm_abs >= M2006_LOAD_START_ROLLER_RPM) {
    g_m2006_load_enabled = true;
  }
  g_m2006_load_pressed = roller_button_pressed && g_m2006_load_enabled;
  g_m2006_forward_pressed =
      DebouncedInput_IsActive(&g_m2006_forward_button, now);
  g_m2006_reverse_pressed =
      DebouncedInput_IsActive(&g_m2006_reverse_button, now);
  g_m2006_limit_reverse_active =
      DebouncedInput_IsActive(&g_m2006_limit_reverse, now);
  g_m2006_limit_forward_active =
      DebouncedInput_IsActive(&g_m2006_limit_forward, now);
}

static bool M2006App_IsFeedbackFresh(int motor_id, uint32_t now)
{
  if (motor_id < M2006_REPLACEMENT_ID || motor_id > M2006_FEED_2_ID) {
    return false;
  }

  const M2006FeedbackState* feedback =
      &g_m2006_feedback[motor_id - M2006_REPLACEMENT_ID];
  return feedback->valid &&
         (uint32_t)(now - feedback->last_update_tick) <=
             M2006_FEEDBACK_TIMEOUT_MS;
}

static void M2006App_SetAllOutputsZero(void)
{
  g_m2006_replacement_output = 0;
  g_m2006_feed_1_output = 0;
  g_m2006_feed_2_output = 0;
  om_rm_set_output(&g_m2006, 0, M2006_REPLACEMENT_ID);
  om_rm_set_output(&g_m2006, 0, M2006_FEED_1_ID);
  om_rm_set_output(&g_m2006, 0, M2006_FEED_2_ID);
}

static void M2006App_Control(uint32_t now)
{
  const bool any_motion_button = g_m2006_load_pressed ||
                                 g_m2006_forward_pressed ||
                                 g_m2006_reverse_pressed ||
                                 g_m2006_auto_reverse_wait_active ||
                                 g_m2006_auto_reverse_active;
  const bool any_operation_button = g_m2006_common_button_pressed ||
                                    g_m2006_forward_pressed ||
                                    g_m2006_reverse_pressed ||
                                    g_m2006_auto_reverse_wait_active ||
                                    g_m2006_auto_reverse_active;
  if (any_motion_button && !g_m2006_motion_was_active) {
    g_m2006_motion_start_tick = now;
  }
  g_m2006_motion_was_active = any_motion_button;

  const bool required_feedback_fresh =
      M2006App_IsFeedbackFresh(M2006_REPLACEMENT_ID, now) &&
      (!g_m2006_load_pressed ||
       (M2006App_IsFeedbackFresh(M2006_FEED_1_ID, now) &&
        M2006App_IsFeedbackFresh(M2006_FEED_2_ID, now)));
  const bool feedback_startup_grace =
      any_motion_button &&
      (uint32_t)(now - g_m2006_motion_start_tick) <
          M2006_FEEDBACK_STARTUP_GRACE_MS;

  /*
   * M2006のフィードバックは、VESC 2台と同じCAN1上で一時的に遅れることが
   * あります。デフォルトでは警告に留め、装填動作を急に0指令へ落としません。
   * M2006_FEEDBACK_FAULT_STOP_ENABLEを1にすると従来通り安全停止します。
   */
  if (any_motion_button && !required_feedback_fresh &&
      !feedback_startup_grace) {
    if (!g_m2006_feedback_timeout_reported) {
      SerialMonitor_Write("M2006=feedback-timeout\r\n");
      g_m2006_feedback_timeout_reported = true;
    }
    if (M2006_FEEDBACK_FAULT_STOP_ENABLE != 0U) {
      g_m2006_feedback_fault_active = true;
    }
  }

  if (!any_operation_button) {
    g_m2006_feedback_fault_active = false;
    g_m2006_feedback_timeout_reported = false;
    g_m2006_tx_fault_reported = false;
  }

  if (g_m2006_feedback_fault_active || g_m2006_tx_fault_active) {
    M2006App_SetAllOutputsZero();
    om_pid_reset(&g_m2006_replacement_pid);
    om_pid_reset(&g_m2006_feed_1_pid);
    om_pid_reset(&g_m2006_feed_2_pid);
    g_m2006_previous_replacement_direction = 0;
    g_m2006_load_was_pressed = g_m2006_load_pressed;
    g_m2006_auto_reverse_wait_active = false;
    g_m2006_auto_reverse_active = false;
    g_m2006_auto_reverse_finished = false;
    g_m2006_motion_was_active = any_motion_button;

    const bool zero_sent = M2006App_SendCommands();
    if (!any_operation_button && zero_sent) {
      g_m2006_tx_fault_active = false;
      g_m2006_can_tx_consecutive_failures = 0U;
    }
    return;
  }

  /* 共通入力の開始時にID5/6の積分値を消します。 */
  if (g_m2006_load_pressed && !g_m2006_load_was_pressed) {
    om_pid_reset(&g_m2006_feed_1_pid);
    om_pid_reset(&g_m2006_feed_2_pid);
  }

  if (g_m2006_load_pressed) {
    const float feed_1_current = om_pid_calc(
        &g_m2006_feed_1_pid,
        M2006_FEED_TARGET_SIGN * M2006_FEED_1_TARGET_RPM,
        (float)om_rm_get_rpm(&g_m2006, M2006_FEED_1_ID),
        CONTROL_PERIOD_SEC);
    const float feed_2_current = om_pid_calc(
        &g_m2006_feed_2_pid,
        M2006_FEED_TARGET_SIGN * M2006_FEED_2_TARGET_RPM,
        (float)om_rm_get_rpm(&g_m2006, M2006_FEED_2_ID),
        CONTROL_PERIOD_SEC);
    g_m2006_feed_1_output = (int16_t)feed_1_current;
    g_m2006_feed_2_output = (int16_t)feed_2_current;
  } else {
    g_m2006_feed_1_output = 0;
    g_m2006_feed_2_output = 0;
    if (g_m2006_load_was_pressed) {
      om_pid_reset(&g_m2006_feed_1_pid);
      om_pid_reset(&g_m2006_feed_2_pid);
    }
  }
  g_m2006_load_was_pressed = g_m2006_load_pressed;

  if (g_m2006_auto_reverse_active && g_m2006_limit_reverse_active) {
    g_m2006_auto_reverse_active = false;
    g_m2006_auto_reverse_finished = true;
  } else if (g_m2006_auto_reverse_wait_active) {
    if (g_m2006_limit_reverse_active) {
      g_m2006_auto_reverse_wait_active = false;
      g_m2006_auto_reverse_finished = true;
    } else if ((uint32_t)(now - g_m2006_auto_reverse_wait_start_tick) >=
               M2006_AUTO_REVERSE_STOP_MS) {
      g_m2006_auto_reverse_wait_active = false;
      g_m2006_auto_reverse_active = true;
      g_m2006_auto_reverse_finished = false;
    }
  } else if (g_m2006_limit_forward_active &&
             (g_m2006_load_pressed || g_m2006_forward_pressed)) {
    g_m2006_auto_reverse_wait_active = true;
    g_m2006_auto_reverse_wait_start_tick = now;
    g_m2006_auto_reverse_active = false;
    g_m2006_auto_reverse_finished = false;
  }

  if (!g_m2006_common_button_pressed &&
      !g_m2006_auto_reverse_wait_active &&
      !g_m2006_auto_reverse_active) {
    g_m2006_auto_reverse_finished = false;
  }

  /*
   * PB0到達後は3秒停止してから自動逆転します。
   * 通常時はPA1/D6/共通入力の順でID4を動かします。
   */
  int8_t replacement_direction = 0;
  if (g_m2006_auto_reverse_active) {
    replacement_direction = -1;
  } else if (g_m2006_auto_reverse_wait_active) {
    replacement_direction = 0;
  } else if (g_m2006_auto_reverse_finished && g_m2006_load_pressed) {
    replacement_direction = 0;
  } else if (g_m2006_forward_pressed) {
    replacement_direction = 1;
  } else if (g_m2006_reverse_pressed) {
    replacement_direction = -1;
  } else if (g_m2006_load_pressed) {
    replacement_direction = 1;
  }

  /* D7は逆転、D8は正転を禁止します。両方作動時は全方向を禁止します。 */
  if ((g_m2006_limit_reverse_active && replacement_direction < 0) ||
      (g_m2006_limit_forward_active && replacement_direction > 0) ||
      (g_m2006_limit_reverse_active && g_m2006_limit_forward_active)) {
    replacement_direction = 0;
  }

  if (replacement_direction != g_m2006_previous_replacement_direction) {
    om_pid_reset(&g_m2006_replacement_pid);
  }

  if (replacement_direction != 0) {
    const float replacement_target_rpm =
        (replacement_direction > 0)
            ? M2006_REPLACEMENT_FORWARD_TARGET_RPM
            : M2006_REPLACEMENT_REVERSE_TARGET_RPM;
    const float replacement_current = om_pid_calc(
        &g_m2006_replacement_pid,
        replacement_target_rpm * (float)replacement_direction,
        (float)om_rm_get_rpm(&g_m2006, M2006_REPLACEMENT_ID),
        CONTROL_PERIOD_SEC);
    g_m2006_replacement_output = (int16_t)replacement_current;
  } else {
    g_m2006_replacement_output = 0;
  }
  g_m2006_previous_replacement_direction = replacement_direction;

  om_rm_set_output(&g_m2006, g_m2006_replacement_output,
                   M2006_REPLACEMENT_ID);
  om_rm_set_output(&g_m2006, g_m2006_feed_1_output, M2006_FEED_1_ID);
  om_rm_set_output(&g_m2006, g_m2006_feed_2_output, M2006_FEED_2_ID);

  if (M2006App_SendCommands()) {
    g_m2006_can_tx_consecutive_failures = 0U;
  } else {
    if (g_m2006_can_tx_consecutive_failures < UINT8_MAX) {
      g_m2006_can_tx_consecutive_failures++;
    }
    if (g_m2006_can_tx_consecutive_failures >=
        M2006_CAN_TX_FAILURE_LIMIT) {
      if (!g_m2006_tx_fault_reported) {
        SerialMonitor_Write("M2006=CAN-TX-fault\r\n");
        g_m2006_tx_fault_reported = true;
      }
      if (M2006_CAN_TX_FAULT_STOP_ENABLE != 0U) {
        g_m2006_tx_fault_active = true;
        M2006App_SetAllOutputsZero();
        (void)M2006App_SendCommands();
      }
    }
  }
}

static bool M2006App_SendCommands(void)
{
  uint8_t group_1[8];
  uint8_t group_2[8];
  const RobomasCore* core = om_rm_get_core_const(&g_m2006);
  om_rm_core_get_output_group(core, group_1, 0U);
  om_rm_core_get_output_group(core, group_2, 1U);

  const bool group_1_sent = M2006App_SendStandardFrame(0x200U, group_1);
  const bool group_2_sent = M2006App_SendStandardFrame(0x1FFU, group_2);
  return group_1_sent && group_2_sent;
}

static bool M2006App_SendStandardFrame(uint32_t can_id,
                                       const uint8_t data[8])
{
  CAN_TxHeaderTypeDef header;
  uint32_t mailbox = 0U;
  memset(&header, 0, sizeof(header));
  header.StdId = can_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  const uint32_t retry_start_tick = HAL_GetTick();
  do {
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0U &&
        HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t*)data,
                            &mailbox) == HAL_OK) {
      return true;
    }
  } while ((uint32_t)(HAL_GetTick() - retry_start_tick) <
           VESC_CAN_TX_RETRY_TIMEOUT_MS);

  return false;
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
    VescApp_ParseStatus1(&msg, now);
    VescApp_ParseStatus5(&msg, now);
    (void)om_vesc_core_parse(&g_vesc, msg.id, msg.data);
    const int m2006_index = om_rm_parse(&g_m2006, msg.id, msg.data);
    if (m2006_index >= (M2006_REPLACEMENT_ID - 1) &&
        m2006_index <= (M2006_FEED_2_ID - 1)) {
      M2006FeedbackState* feedback =
          &g_m2006_feedback[m2006_index - (M2006_REPLACEMENT_ID - 1)];
      feedback->last_update_tick = now;
      feedback->valid = true;
    }
  }

  if (can_cube_get_rx_overflow_count(&g_can_cube) != 0U) {
    g_rx_overflow_latched = true;
  }
}

static void Can2RemoteTrigger_Init(void)
{
  CAN_FilterTypeDef filter;
  memset(&filter, 0, sizeof(filter));

  /*
   * bxCANの32 bitフィルタでは標準IDを21 bit左へ配置します。
   * IDE/RTRもマスクし、標準ID 200のデータフレームだけをFIFO0へ通します。
   * フィルタバンク0～13はCAN1、14～27はCAN2が使用します。
   */
  filter.FilterBank = CAN2_REMOTE_TRIGGER_FILTER_BANK;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (CAN2_REMOTE_TRIGGER_STD_ID << 5) & 0xFFFFU;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0xFFE0U;
  filter.FilterMaskIdLow = 0x0006U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = CAN2_REMOTE_TRIGGER_FILTER_BANK;

  if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK ||
      HAL_CAN_Start(&hcan2) != HAL_OK) {
    Error_Handler();
  }
}

static void Can2RemoteTrigger_Poll(uint32_t now)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) != 0U) {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      break;
    }

    if (header.IDE == CAN_ID_STD && header.RTR == CAN_RTR_DATA &&
        header.StdId == CAN2_REMOTE_TRIGGER_STD_ID) {
      if (header.DLC >= 1U && data[0] == CAN2_REMOTE_TRIGGER_ON_VALUE) {
        g_can2_remote_trigger_last_rx_tick = now;
        g_can2_remote_trigger_valid = true;
        g_can2_remote_trigger_active = true;
      } else if (header.DLC >= 1U &&
                 data[0] == CAN2_REMOTE_TRIGGER_OFF_VALUE) {
        g_can2_remote_trigger_last_rx_tick = now;
        g_can2_remote_trigger_valid = true;
        g_can2_remote_trigger_active = false;
      }
    }
  }

  if (g_can2_remote_trigger_valid &&
      (uint32_t)(now - g_can2_remote_trigger_last_rx_tick) >
          CAN2_REMOTE_TRIGGER_TIMEOUT_MS) {
    g_can2_remote_trigger_active = false;
  }
}

static bool Can2RemoteTrigger_IsActive(uint32_t now)
{
  if (!g_can2_remote_trigger_valid ||
      (uint32_t)(now - g_can2_remote_trigger_last_rx_tick) >
          CAN2_REMOTE_TRIGGER_TIMEOUT_MS) {
    g_can2_remote_trigger_active = false;
  }
  return g_can2_remote_trigger_active;
}

static void VescApp_ParseStatus1(const CanMessage* msg, uint32_t now)
{
  if (msg == NULL || msg->len < 8U) {
    return;
  }

  const uint32_t packet_id = (msg->id >> 8) & 0xFFU;
  if (packet_id != VESC_CAN_PACKET_STATUS_1_ID) {
    return;
  }

  const uint8_t controller_id = (uint8_t)(msg->id & 0xFFU);
  VescStatus1Data* status = NULL;
  if (controller_id == (uint8_t)VESC_MOTOR_1_CAN_ID) {
    status = &g_vesc_motor_1_status1;
  } else if (controller_id == (uint8_t)VESC_MOTOR_2_CAN_ID) {
    status = &g_vesc_motor_2_status1;
  } else {
    return;
  }

  status->electrical_rpm =
      (int32_t)(((uint32_t)msg->data[0] << 24) |
                ((uint32_t)msg->data[1] << 16) |
                ((uint32_t)msg->data[2] << 8) |
                (uint32_t)msg->data[3]);
  status->motor_current_x10 =
      (int16_t)(((uint16_t)msg->data[4] << 8) |
                (uint16_t)msg->data[5]);
  status->duty_x1000 =
      (int16_t)(((uint16_t)msg->data[6] << 8) |
                (uint16_t)msg->data[7]);
  status->last_update_tick = now;
  status->valid = true;

  /* 目標付近でdutyを保持している間だけ平滑化します。 */
  if (g_app_state == VESC_APP_STARTING_DUTY &&
      g_startup_rpm_in_range) {
    const float current_a = (float)status->motor_current_x10 * 0.1f;
    if (status->duty_sample_count == 0U) {
      status->duty_filtered_current_a = current_a;
    } else {
      status->duty_filtered_current_a +=
          ROLLER_DUTY_CURRENT_FILTER_ALPHA *
          (current_a - status->duty_filtered_current_a);
    }
    if (status->duty_sample_count < UINT16_MAX) {
      status->duty_sample_count++;
    }
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

static bool VescApp_IsStatus1Fresh(const VescStatus1Data* status,
                                   uint32_t now)
{
  return status != NULL && status->valid &&
         (uint32_t)(now - status->last_update_tick) <=
             VESC_STATUS_1_TIMEOUT_MS;
}

static void VescApp_ResetDutyCurrentFilters(void)
{
  g_vesc_motor_1_status1.duty_filtered_current_a = 0.0f;
  g_vesc_motor_1_status1.duty_sample_count = 0U;
  g_vesc_motor_2_status1.duty_filtered_current_a = 0.0f;
  g_vesc_motor_2_status1.duty_sample_count = 0U;
}

static bool VescApp_IsDutyFeedforwardReady(uint32_t now)
{
  (void)now;
  return true;
}

static float VescApp_GetStartupFeedforwardCurrent(uint32_t now)
{
  (void)now;
  const float direction = (ROLLER_TARGET_RPM < 0.0f) ? -1.0f : 1.0f;
  float current_a = ROLLER_FEEDFORWARD_CURRENT_A;
  if (current_a < 0.0f) {
    current_a = -current_a;
  }
  if (current_a > ROLLER_MAX_CURRENT_A) {
    current_a = ROLLER_MAX_CURRENT_A;
  }
  return direction * current_a;
}

static float VescApp_GetTargetFeedforwardLimit(void)
{
  float current_a = ROLLER_FEEDFORWARD_CURRENT_A;
  if (current_a < 0.0f) {
    current_a = -current_a;
  }
  if (current_a > ROLLER_MAX_CURRENT_A) {
    current_a = ROLLER_MAX_CURRENT_A;
  }
  return current_a;
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
  const int16_t raw_delta_count =
      (int16_t)(count - g_roller_encoder.previous_count);
  const int16_t delta_count =
      (int16_t)(raw_delta_count * ROLLER_ENCODER_DIRECTION);
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
    const uint16_t next_head =
        (uint16_t)((g_serial_tx_head + 1U) % SERIAL_TX_BUFFER_SIZE);
    if (next_head == g_serial_tx_tail) {
      return;
    }
    g_serial_tx_buffer[g_serial_tx_head] = *text;
    g_serial_tx_head = next_head;
    text++;
  }
}

static void SerialMonitor_ServiceTx(void)
{
  if (g_serial_tx_tail == g_serial_tx_head ||
      (USART2->SR & USART_SR_TXE) == 0U) {
    return;
  }

  USART2->DR = (uint8_t)g_serial_tx_buffer[g_serial_tx_tail];
  g_serial_tx_tail =
      (uint16_t)((g_serial_tx_tail + 1U) % SERIAL_TX_BUFFER_SIZE);
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

  (void)snprintf(line, sizeof(line),
                 ">ROLLER_FEEDFORWARD_CURRENT_mA:%ld\r\n",
                 (long)(g_feedforward_current_a * 1000.0f));
  SerialMonitor_Write(line);

  (void)snprintf(line, sizeof(line),
                 ">ROLLER_FEEDFORWARD_LIMIT_mA:%ld\r\n",
                 (long)(VescApp_GetTargetFeedforwardLimit() * 1000.0f));
  SerialMonitor_Write(line);

  (void)snprintf(line, sizeof(line),
                 ">ROLLER_PID_CORRECTION_CURRENT_mA:%ld\r\n",
                 (long)(g_pid_correction_current_a * 1000.0f));
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

  if (VescApp_IsStatus1Fresh(&g_vesc_motor_1_status1, now)) {
    (void)snprintf(line, sizeof(line),
                   ">VESC_105_MOTOR_CURRENT_mA:%ld\r\n",
                   (long)g_vesc_motor_1_status1.motor_current_x10 * 100L);
    SerialMonitor_Write(line);
  }

  if (VescApp_IsStatus1Fresh(&g_vesc_motor_2_status1, now)) {
    (void)snprintf(line, sizeof(line),
                   ">VESC_112_MOTOR_CURRENT_mA:%ld\r\n",
                   (long)g_vesc_motor_2_status1.motor_current_x10 * 100L);
    SerialMonitor_Write(line);
  }

  if (VescApp_IsStatus1Fresh(&g_vesc_motor_1_status1, now) &&
      VescApp_IsStatus1Fresh(&g_vesc_motor_2_status1, now)) {
    const long average_motor_current_ma =
        ((long)g_vesc_motor_1_status1.motor_current_x10 +
         (long)g_vesc_motor_2_status1.motor_current_x10) * 50L;
    (void)snprintf(line, sizeof(line),
                   ">VESC_AVERAGE_MOTOR_CURRENT_mA:%ld\r\n",
                   average_motor_current_ma);
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

  /* 装填用C610/M2006の回転数と、各C610へ送る電流指令値です。 */
  (void)snprintf(line, sizeof(line), ">M2006_ID4_RPM:%d\r\n",
                 (int)om_rm_get_rpm(&g_m2006, M2006_REPLACEMENT_ID));
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_ID5_RPM:%d\r\n",
                 (int)om_rm_get_rpm(&g_m2006, M2006_FEED_1_ID));
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_ID6_RPM:%d\r\n",
                 (int)om_rm_get_rpm(&g_m2006, M2006_FEED_2_ID));
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_ID4_CURRENT:%d\r\n",
                 (int)g_m2006_replacement_output);
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_ID5_CURRENT:%d\r\n",
                 (int)g_m2006_feed_1_output);
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_ID6_CURRENT:%d\r\n",
                 (int)g_m2006_feed_2_output);
  SerialMonitor_Write(line);
  (void)snprintf(line, sizeof(line), ">M2006_LOAD_BUTTON:%u\r\n",
                 g_m2006_load_pressed ? 1U : 0U);
  SerialMonitor_Write(line);

  (void)snprintf(line, sizeof(line), ">CAN2_ID200_BUTTON:%u\r\n",
                 g_can2_remote_trigger_active ? 1U : 0U);
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

static bool VescApp_IsEncoderDirectionWrong(void)
{
  const int32_t rpm = g_roller_encoder.rpm;
  if (ROLLER_TARGET_RPM > 0.0f) {
    return rpm <= -ROLLER_ENCODER_DIRECTION_CHECK_RPM;
  }
  if (ROLLER_TARGET_RPM < 0.0f) {
    return rpm >= ROLLER_ENCODER_DIRECTION_CHECK_RPM;
  }
  return false;
}

static bool VescApp_IsRollerOverspeed(void)
{
  int32_t rpm_abs = g_roller_encoder.rpm;
  int32_t target_abs = (int32_t)ROLLER_TARGET_RPM;
  if (rpm_abs < 0) {
    rpm_abs = -rpm_abs;
  }
  if (target_abs < 0) {
    target_abs = -target_abs;
  }
  return rpm_abs >= (target_abs + ROLLER_OVERSPEED_MARGIN_RPM);
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
  g_pid_correction_current_a = 0.0f;
  g_feedforward_current_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_duty_hold_command = 0.0f;
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
  g_pid_correction_current_a = 0.0f;
  g_feedforward_current_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_duty_hold_command = 0.0f;
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
  g_pid_correction_current_a = 0.0f;
  g_feedforward_current_a = 0.0f;
  g_pid_current_ramp_active = false;
  g_startup_duty_command = 0.0f;
  g_startup_duty_hold_command = 0.0f;
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
      g_startup_timeout_fault_latched ||
      g_m2006_feedback_fault_active || g_m2006_tx_fault_active) {
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
