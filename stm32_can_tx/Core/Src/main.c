/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : F446RE CAN1 250 kbps + BNO085 IMU + HC-SR04 + LED + Switch
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define H723_REQUEST_ID      0x201U
#define F446_RESPONSE_ID     0x281U
#define IMU_DATA_ID          0x301U
#define LED_CMD_ID           0x310U   /* H723 -> F446: 좌/우/브레이크 LED 명령 */
#define SWITCH_STATUS_ID     0x311U   /* F446 -> H723: 토글스위치 상태 */
#define ULTRASONIC_ID        0x312U   /* F446 -> H723: 초음파 거리(mm) */

#define CAN_PAYLOAD_LENGTH   8U
#define BNO085_ADDR          (0x4A << 1)

#ifndef BNO_RST_Pin
#define BNO_RST_Pin         GPIO_PIN_4
#define BNO_RST_GPIO_Port   GPIOB
#endif

#ifndef BNO_INT_Pin
#define BNO_INT_Pin         GPIO_PIN_5
#define BNO_INT_GPIO_Port   GPIOB
#endif

#ifndef LEFT_LED_Pin
#define LEFT_LED_Pin        GPIO_PIN_0
#define LEFT_LED_GPIO_Port  GPIOA
#endif

#ifndef RIGHT_LED_Pin
#define RIGHT_LED_Pin       GPIO_PIN_1
#define RIGHT_LED_GPIO_Port GPIOA
#endif

#ifndef BRAKE_LED_Pin
#define BRAKE_LED_Pin       GPIO_PIN_4
#define BRAKE_LED_GPIO_Port GPIOA
#endif

#ifndef SWITCH_INPUT_Pin
#define SWITCH_INPUT_Pin        GPIO_PIN_3
#define SWITCH_INPUT_GPIO_Port  GPIOC
#endif

#ifndef US_TRIG_Pin
#define US_TRIG_Pin         GPIO_PIN_10
#define US_TRIG_GPIO_Port   GPIOB
#endif

#define TURN_SIGNAL_PERIOD_MS   500U   /* 방향지시등 깜빡임 주기 */
#define ULTRASONIC_PERIOD_MS    60U    /* 초음파 측정 주기 (음속 왕복 여유 포함) */
#define SWITCH_TX_PERIOD_MS     100U   /* 스위치 상태 전송 주기 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static CAN_RxHeaderTypeDef RxHeader;
static CAN_TxHeaderTypeDef TxHeader;
static CAN_TxHeaderTypeDef ImuTxHeader;
static CAN_TxHeaderTypeDef SwitchTxHeader;
static CAN_TxHeaderTypeDef UltrasonicTxHeader;

static uint8_t RxData[8];
static uint8_t TxData[8];
static uint8_t ImuTxData[8];
static uint8_t SwitchTxData[8];
static uint8_t UltrasonicTxData[8];

static uint8_t bno_seq_ctrl = 0;
static uint32_t imu_last_tick = 0;

/* LED 명령 상태 (H723로부터 수신) */
volatile uint8_t led_left_cmd = 0;
volatile uint8_t led_right_cmd = 0;
volatile uint8_t led_brake_cmd = 0;

static uint32_t turn_signal_last_tick = 0;
static uint8_t  turn_signal_phase = 0;   /* 0=off, 1=on (깜빡임용 토글) */

/* 스위치 */
static uint32_t switch_last_tick = 0;
volatile uint8_t switch_pressed = 0;   /* 1=눌림, 0=안눌림 */

/* HC-SR04 */
static uint32_t ultrasonic_last_tick = 0;
volatile uint32_t ultrasonic_distance_mm = 0;
volatile uint8_t  ultrasonic_echo_captured = 0;
static volatile uint32_t echo_rise_tick = 0;
static volatile uint8_t  echo_state = 0;   /* 0=idle, 1=waiting falling */

/* Live Expressions용 진단 변수 */
volatile uint32_t can250_rx_count = 0;
volatile uint32_t can250_rx_ok_count = 0;
volatile uint32_t can250_rx_bad_count = 0;
volatile uint32_t can250_tx_count = 0;
volatile uint32_t can250_tx_busy_count = 0;
volatile uint32_t can250_tx_error_count = 0;
volatile uint32_t can250_last_error = 0;
volatile uint32_t can250_last_received_counter = 0;

volatile uint32_t imu_tx_count = 0;
volatile uint32_t imu_tx_error_count = 0;
volatile HAL_StatusTypeDef imu_i2c_ready_status;
volatile HAL_StatusTypeDef imu_enable_status;

volatile uint16_t dbg_last_len = 0;
volatile uint8_t  dbg_last_channel = 0xFF;
volatile uint8_t  dbg_last_reportid = 0xFF;
volatile uint32_t dbg_poll_count = 0;
volatile uint32_t dbg_zero_len_count = 0;
volatile uint32_t dbg_match_count = 0;
volatile uint32_t dbg_int_wait_timeout_count = 0;

volatile uint32_t led_cmd_rx_count = 0;
volatile uint32_t switch_tx_count = 0;
volatile uint32_t ultrasonic_tx_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void CAN250_Start(void);
static uint32_t ReadU32LE(const uint8_t *data);

static void BNO_GPIO_Init(void);
static void BNO_HardwareReset(void);
static uint8_t BNO_WaitForInt(uint32_t timeout_ms);

static uint16_t BNO_ReadPacket(uint8_t *buf, uint16_t maxlen);
static void BNO_EnableRotationVector(void);
static void IMU_Init(void);
static void IMU_PollAndSend(void);

static void LED_Apply(uint8_t left, uint8_t right, uint8_t brake);
static void LED_TurnSignalUpdate(void);

static void Switch_PollAndSend(void);

static void Ultrasonic_Trigger(void);
static void Ultrasonic_PollAndSend(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t ReadU32LE(const uint8_t *data)
{
  return ((uint32_t)data[0]) |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static void CAN250_Start(void)
{
  CAN_FilterTypeDef sFilterConfig = {0};

  /*
   * 0x201(에코 요청), 0x310(LED 명령) 둘 다 받아야 하니까
   * Mask 필터로 넓게 걸어서 0x200~0x3FF 대역을 전부 FIFO0으로 받고
   * 콜백에서 StdId로 구분한다.
   */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000U;
  sFilterConfig.FilterIdLow = 0x0000U;
  sFilterConfig.FilterMaskIdHigh = 0x0000U;
  sFilterConfig.FilterMaskIdLow = 0x0000U;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* F446RE -> H723 에코 응답 */
  TxHeader.StdId = F446_RESPONSE_ID;
  TxHeader.ExtId = 0U;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = CAN_PAYLOAD_LENGTH;

  /* F446RE -> H723 IMU 데이터 */
  ImuTxHeader.StdId = IMU_DATA_ID;
  ImuTxHeader.ExtId = 0U;
  ImuTxHeader.IDE = CAN_ID_STD;
  ImuTxHeader.RTR = CAN_RTR_DATA;
  ImuTxHeader.DLC = CAN_PAYLOAD_LENGTH;

  /* F446RE -> H723 스위치 상태 */
  SwitchTxHeader.StdId = SWITCH_STATUS_ID;
  SwitchTxHeader.ExtId = 0U;
  SwitchTxHeader.IDE = CAN_ID_STD;
  SwitchTxHeader.RTR = CAN_RTR_DATA;
  SwitchTxHeader.DLC = CAN_PAYLOAD_LENGTH;

  /* F446RE -> H723 초음파 거리 */
  UltrasonicTxHeader.StdId = ULTRASONIC_ID;
  UltrasonicTxHeader.ExtId = 0U;
  UltrasonicTxHeader.IDE = CAN_ID_STD;
  UltrasonicTxHeader.RTR = CAN_RTR_DATA;
  UltrasonicTxHeader.DLC = CAN_PAYLOAD_LENGTH;

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(
          &hcan1,
          CAN_IT_RX_FIFO0_MSG_PENDING |
          CAN_IT_ERROR |
          CAN_IT_BUSOFF) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ===================== BNO085 IMU ===================== */

static void BNO_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = BNO_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BNO_RST_GPIO_Port, &GPIO_InitStruct);

  /* INT는 CubeMX에서 EXTI로 이미 설정됨 (MX_GPIO_Init에서 처리) */
}

static void BNO_HardwareReset(void)
{
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BNO_RST_GPIO_Port, BNO_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
}

static uint8_t BNO_WaitForInt(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      dbg_int_wait_timeout_count++;
      return 0;
    }
  }
  return 1;
}

static uint16_t BNO_ReadPacket(uint8_t *buf, uint16_t maxlen)
{
  if (HAL_I2C_Master_Receive(&hi2c1, BNO085_ADDR, buf, maxlen, 50) != HAL_OK)
    return 0;

  uint16_t len = ((buf[1] & 0x7F) << 8) | buf[0];
  if (len == 0) return 0;

  return (len > maxlen) ? maxlen : len;
}

static void BNO_EnableRotationVector(void)
{
  uint8_t pkt[21] = {0};
  uint16_t len = 21;

  pkt[0] = len & 0xFF;
  pkt[1] = (len >> 8) & 0x7F;
  pkt[2] = 2;
  pkt[3] = bno_seq_ctrl++;

  pkt[4] = 0xFD;
  pkt[5] = 0x05;
  pkt[6] = 0;
  pkt[7] = 0; pkt[8] = 0;
  pkt[9]  = 0x20; pkt[10] = 0x4E; pkt[11] = 0x00; pkt[12] = 0x00;

  imu_enable_status = HAL_I2C_Master_Transmit(&hi2c1, BNO085_ADDR, pkt, len, 100);
}

static void IMU_Init(void)
{
  uint8_t dump[32];

  BNO_GPIO_Init();
  BNO_HardwareReset();

  imu_i2c_ready_status = HAL_I2C_IsDeviceReady(&hi2c1, BNO085_ADDR, 3, 100);
  if (imu_i2c_ready_status != HAL_OK) return;

  for (int i = 0; i < 30; i++)
  {
    if (BNO_WaitForInt(50))
    {
      BNO_ReadPacket(dump, sizeof(dump));
    }
    HAL_Delay(5);
  }

  BNO_EnableRotationVector();
  HAL_Delay(100);
}

static void IMU_PollAndSend(void)
{
  uint8_t rx[32];
  uint32_t txMailbox;

  if (HAL_GPIO_ReadPin(BNO_INT_GPIO_Port, BNO_INT_Pin) == GPIO_PIN_SET)
  {
    return;
  }

  uint16_t len = BNO_ReadPacket(rx, sizeof(rx));

  dbg_poll_count++;
  dbg_last_len = len;

  if (len == 0)
  {
    dbg_zero_len_count++;
    return;
  }

  dbg_last_channel = rx[2];

  if (len >= 22 && rx[2] == 3 && rx[4] == 0xFB && rx[9] == 0x05)
  {
    dbg_last_reportid = rx[9];
    dbg_match_count++;

    ImuTxData[0] = rx[14]; ImuTxData[1] = rx[15];
    ImuTxData[2] = rx[16]; ImuTxData[3] = rx[17];
    ImuTxData[4] = rx[18]; ImuTxData[5] = rx[19];
    ImuTxData[6] = rx[20]; ImuTxData[7] = rx[21];

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return;

    if (HAL_CAN_AddTxMessage(&hcan1, &ImuTxHeader, ImuTxData, &txMailbox) == HAL_OK)
      imu_tx_count++;
    else
      imu_tx_error_count++;
  }
}

/* ===================== LED (좌/우 방향지시등 + 브레이크등) ===================== */

/*
 * H723로부터 0x310 수신 시 led_left_cmd/led_right_cmd/led_brake_cmd에 반영됨.
 * 좌/우는 "켜라(1)" 명령이 오면 F446RE 자체적으로 500ms 주기 깜빡임,
 * 브레이크는 단순 on/off.
 */
static void LED_Apply(uint8_t left_on, uint8_t right_on, uint8_t brake_on)
{
  HAL_GPIO_WritePin(LEFT_LED_GPIO_Port, LEFT_LED_Pin, left_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_LED_GPIO_Port, RIGHT_LED_Pin, right_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BRAKE_LED_GPIO_Port, BRAKE_LED_Pin, brake_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void LED_TurnSignalUpdate(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - turn_signal_last_tick) >= TURN_SIGNAL_PERIOD_MS)
  {
    turn_signal_last_tick = now;
    turn_signal_phase = !turn_signal_phase;
  }

  uint8_t left_out  = (led_left_cmd  && turn_signal_phase) ? 1U : 0U;
  uint8_t right_out = (led_right_cmd && turn_signal_phase) ? 1U : 0U;
  uint8_t brake_out = led_brake_cmd ? 1U : 0U;   /* 브레이크등은 깜빡임 없이 그대로 */

  LED_Apply(left_out, right_out, brake_out);
}

/* ===================== 토글 스위치 ===================== */

static void Switch_PollAndSend(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t txMailbox;

  /* SWITCH_INPUT_Pin: Pull-up 설정이므로 안 눌렸을 때 High, 눌리면 Low */
  switch_pressed = (HAL_GPIO_ReadPin(SWITCH_INPUT_GPIO_Port, SWITCH_INPUT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if ((now - switch_last_tick) < SWITCH_TX_PERIOD_MS) return;
  switch_last_tick = now;

  memset(SwitchTxData, 0, sizeof(SwitchTxData));
  SwitchTxData[0] = switch_pressed;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return;

  if (HAL_CAN_AddTxMessage(&hcan1, &SwitchTxHeader, SwitchTxData, &txMailbox) == HAL_OK)
  {
    switch_tx_count++;
  }
}

/* ===================== HC-SR04 초음파 ===================== */

static void Ultrasonic_Trigger(void)
{
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
  /* 짧은 정지 후 10us 펄스 */
  for (volatile int i = 0; i < 5; i++) { __NOP(); }
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 170; i++) { __NOP(); }  /* 대략 10us @ 84MHz core */
  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

  echo_state = 0;
  ultrasonic_echo_captured = 0;

  /* TIM4 Input Capture 인터럽트 시작 (main에서 이미 HAL_TIM_IC_Start_IT 했다면 재호출 불필요) */
}

static void Ultrasonic_PollAndSend(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t txMailbox;

  if ((now - ultrasonic_last_tick) < ULTRASONIC_PERIOD_MS) return;
  ultrasonic_last_tick = now;

  Ultrasonic_Trigger();

  /* 이전 측정 결과를 CAN으로 송신 (측정은 인터럽트 콜백에서 비동기 갱신됨) */
  memset(UltrasonicTxData, 0, sizeof(UltrasonicTxData));
  UltrasonicTxData[0] = (uint8_t)(ultrasonic_distance_mm & 0xFF);
  UltrasonicTxData[1] = (uint8_t)((ultrasonic_distance_mm >> 8) & 0xFF);

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return;

  if (HAL_CAN_AddTxMessage(&hcan1, &UltrasonicTxHeader, UltrasonicTxData, &txMailbox) == HAL_OK)
  {
    ultrasonic_tx_count++;
  }
}

/**
 * @brief TIM4 Input Capture 콜백 - HC-SR04 ECHO 펄스 폭 측정
 *        Rising Edge에서 시작 tick 저장 -> Falling Edge로 극성 전환 -> 폭 계산
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    if (echo_state == 0)
    {
      /* Rising Edge 캡처됨 -> Falling Edge로 극성 전환 */
      echo_rise_tick = captured;
      echo_state = 1;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
      /* Falling Edge 캡처됨 -> 펄스 폭(us) 계산, 34.3cm/ms 왕복 기준 mm 환산 */
      uint32_t pulse_us;
      if (captured >= echo_rise_tick)
      {
        pulse_us = captured - echo_rise_tick;
      }
      else
      {
        /* 타이머 오버플로우(65535) 발생 시 */
        pulse_us = (65535U - echo_rise_tick) + captured;
      }

      /* distance(mm) = pulse_us * 0.343 / 2  (음속 343m/s = 0.343mm/us) */
      ultrasonic_distance_mm = (uint32_t)((pulse_us * 343UL) / 2000UL);
      ultrasonic_echo_captured = 1;

      /* 다음 측정을 위해 다시 Rising Edge로 되돌림 */
      echo_state = 0;
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  CAN250_Start();
  IMU_Init();

  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  imu_last_tick = HAL_GetTick();
  turn_signal_last_tick = HAL_GetTick();
  switch_last_tick = HAL_GetTick();
  ultrasonic_last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - imu_last_tick) >= 20U)
    {
      imu_last_tick = now;
      IMU_PollAndSend();
    }

    LED_TurnSignalUpdate();
    Switch_PollAndSend();
    Ultrasonic_PollAndSend();

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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/* USER CODE BEGIN 4 */

/**
  * @brief CAN1 RX FIFO0 message pending callback
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t txMailbox;
  uint32_t i;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    can250_rx_bad_count++;
    can250_last_error = HAL_CAN_GetError(hcan);
    return;
  }

  can250_rx_count++;

  if (RxHeader.IDE != CAN_ID_STD)
  {
    can250_rx_bad_count++;
    return;
  }

  /* ===== LED 명령 (0x310) ===== */
  if (RxHeader.StdId == LED_CMD_ID)
  {
    led_left_cmd  = RxData[0];
    led_right_cmd = RxData[1];
    led_brake_cmd = RxData[2];
    led_cmd_rx_count++;
    return;
  }

  /* ===== 기존 에코 요청 (0x201) ===== */
  if ((RxHeader.RTR != CAN_RTR_DATA) ||
      (RxHeader.StdId != H723_REQUEST_ID) ||
      (RxHeader.DLC != CAN_PAYLOAD_LENGTH) ||
      (RxData[0] != 0xA5U) ||
      (RxData[1] != 0x5AU) ||
      (RxData[2] != 0x02U) ||
      (RxData[3] != 0x01U))
  {
    can250_rx_bad_count++;
    return;
  }

  can250_rx_ok_count++;
  can250_last_received_counter = ReadU32LE(&RxData[4]);

  for (i = 0U; i < CAN_PAYLOAD_LENGTH; i++)
  {
    TxData[i] = RxData[i];
  }

  TxData[0] = 0x5AU;
  TxData[1] = 0xA5U;
  TxData[2] = 0x02U;
  TxData[3] = 0x02U;

  if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
  {
    can250_tx_busy_count++;
    return;
  }

  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &txMailbox) == HAL_OK)
  {
    can250_tx_count++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
  else
  {
    can250_tx_error_count++;
    can250_last_error = HAL_CAN_GetError(hcan);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN1)
  {
    can250_last_error = HAL_CAN_GetError(hcan);
  }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
