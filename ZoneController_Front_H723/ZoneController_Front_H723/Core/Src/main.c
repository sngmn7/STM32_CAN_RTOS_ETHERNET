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
#include "lwip.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "front_zone_network.h"
#include "front_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* -------------------------------------------------------------------------- */
/* FDCAN2: Classical CAN 250 kbps                                              */
/* H723 -> F407/PCAN: ID 0x201                                                 */
/* F407 echo response expected: ID 0x281                                       */
/* -------------------------------------------------------------------------- */
static FDCAN_TxHeaderTypeDef can250_tx_header;
static FDCAN_RxHeaderTypeDef can250_rx_header;
static uint8_t can250_tx_data[8];
static uint8_t can250_rx_data[8];

volatile uint32_t can250_next_counter = 0;
volatile uint32_t can250_last_sent_counter = 0;
volatile uint32_t can250_last_received_counter = 0;

volatile uint32_t can250_tx_count = 0;
volatile uint32_t can250_rx_count = 0;
volatile uint32_t can250_rx_ok_count = 0;
volatile uint32_t can250_rx_bad_count = 0;
volatile uint32_t can250_tx_busy_count = 0;
volatile uint32_t can250_tx_error_count = 0;
volatile uint32_t can250_last_error = 0;
volatile uint32_t can250_tx_fifo_free = 0;

volatile FDCAN_ErrorCountersTypeDef can250_error_counters;
volatile FDCAN_ProtocolStatusTypeDef can250_protocol_status;

static uint32_t can250_last_tx_tick = 0;

/* Front F407 split signal frames received by H723 */
volatile uint32_t front_last_rx_id = 0U;
volatile uint32_t front_unexpected_rx_count = 0U;

volatile uint16_t f407_steering_adc = 0U;
volatile uint16_t f407_brake_adc = 0U;
volatile uint16_t f407_accel_adc = 0U;
volatile uint8_t f407_turn_left_output = 0U;
volatile uint8_t f407_turn_right_output = 0U;
volatile uint8_t f407_turn_switch_pressed = 0U;
volatile uint8_t f407_headlamp_output = 0U;
volatile int16_t f407_temperature_x10 = 0;
volatile uint16_t f407_humidity_x10 = 0U;
volatile uint8_t f407_dht11_valid = 0U;
volatile uint16_t f407_actuator_position = 0U;
volatile uint8_t f407_actuator_output_enable = 0U;
volatile uint8_t f407_steering_override_active = 0U;
volatile uint8_t f407_headlamp_override_active = 0U;
volatile uint8_t f407_can_busoff = 0U;

/* Per-ID receive counters for RTOS task separation and diagnostics */
volatile uint32_t f407_steering_rx_count = 0U;
volatile uint32_t f407_brake_rx_count = 0U;
volatile uint32_t f407_accel_rx_count = 0U;
volatile uint32_t f407_turn_left_rx_count = 0U;
volatile uint32_t f407_turn_right_rx_count = 0U;
volatile uint32_t f407_turn_switch_rx_count = 0U;
volatile uint32_t f407_headlamp_output_rx_count = 0U;
volatile uint32_t f407_temperature_rx_count = 0U;
volatile uint32_t f407_humidity_rx_count = 0U;
volatile uint32_t f407_dht11_valid_rx_count = 0U;
volatile uint32_t f407_actuator_position_rx_count = 0U;
volatile uint32_t f407_actuator_enable_rx_count = 0U;
volatile uint32_t f407_steering_override_rx_count = 0U;
volatile uint32_t f407_headlamp_override_rx_count = 0U;
volatile uint32_t f407_busoff_rx_count = 0U;

/* H723 -> Front F407 split command state */
volatile uint8_t front_steering_override_enable = 0U;
volatile uint8_t front_actuator_enable_command = 0U;
volatile uint8_t front_headlamp_override_enable = 0U;
volatile uint8_t front_headlamp_command = 0U;
volatile uint16_t front_actuator_target_command = 0U;
volatile uint8_t front_turn_override_enable = 0U;
volatile uint8_t front_turn_mode_command = 0U;

/* Counts successful insertion into FDCAN Tx FIFO, not bus-level completion. */
volatile uint32_t front_steering_override_tx_count = 0U;
volatile uint32_t front_actuator_enable_tx_count = 0U;
volatile uint32_t front_actuator_target_tx_count = 0U;
volatile uint32_t front_headlamp_override_tx_count = 0U;
volatile uint32_t front_headlamp_command_tx_count = 0U;
volatile uint32_t front_turn_override_tx_count = 0U;
volatile uint32_t front_turn_mode_tx_count = 0U;
volatile uint32_t front_command_tx_busy_count = 0U;
volatile uint32_t front_command_tx_error_count = 0U;
volatile uint32_t front_command_last_error = 0U;

static uint8_t front_command_scheduler_index = 0U;
static uint32_t front_steering_override_last_tx_tick = 0U;
static uint32_t front_actuator_enable_last_tx_tick = 0U;
static uint32_t front_actuator_target_last_tx_tick = 0U;
static uint32_t front_headlamp_override_last_tx_tick = 0U;
static uint32_t front_headlamp_command_last_tx_tick = 0U;
static uint32_t front_turn_override_last_tx_tick = 0U;
static uint32_t front_turn_mode_last_tx_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void CAN250_Start(void);
static void CAN250_SendPing(void);

static HAL_StatusTypeDef FrontCommand_SendU8(uint32_t identifier, uint8_t value);
static HAL_StatusTypeDef FrontCommand_SendU16(uint32_t identifier, uint16_t value);
static void FrontCommand_SchedulerInit(uint32_t now);
static void FrontCommand_SendNext(uint32_t now);

static void CAN_UpdateDiagnostics(void);

static void CAN_WriteU32LE(uint8_t *data, uint32_t value);
static uint32_t CAN_ReadU32LE(const uint8_t *data);
static void EthernetMPU_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CAN250_TX_PERIOD_MS                     500U

/* F407 -> H723: one signal per Standard CAN ID */
#define CAN_ID_F407_STEERING_ADC                 0x210U
#define CAN_ID_F407_BRAKE_ADC                    0x211U
#define CAN_ID_F407_ACCEL_ADC                    0x212U
#define CAN_ID_F407_TURN_LEFT_OUTPUT             0x213U
#define CAN_ID_F407_TURN_RIGHT_OUTPUT            0x214U
#define CAN_ID_F407_TURN_SWITCH_PRESSED          0x215U
#define CAN_ID_F407_HEADLAMP_OUTPUT              0x216U
#define CAN_ID_F407_TEMPERATURE_X10              0x217U
#define CAN_ID_F407_HUMIDITY_X10                 0x218U
#define CAN_ID_F407_DHT11_VALID                  0x219U
#define CAN_ID_F407_ACTUATOR_POSITION            0x21AU
#define CAN_ID_F407_ACTUATOR_ENABLE              0x21BU
#define CAN_ID_F407_STEERING_OVERRIDE_ACTIVE     0x21CU
#define CAN_ID_F407_HEADLAMP_OVERRIDE_ACTIVE     0x21DU
#define CAN_ID_F407_CAN_BUSOFF                    0x21EU
#define CAN_ID_F407_ECHO                         0x281U

/* H723 -> F407: one command per Standard CAN ID */
#define CAN_ID_H723_STEERING_OVERRIDE            0x310U
#define CAN_ID_H723_ACTUATOR_ENABLE              0x311U
#define CAN_ID_H723_ACTUATOR_TARGET              0x312U
#define CAN_ID_H723_HEADLAMP_OVERRIDE            0x313U
#define CAN_ID_H723_HEADLAMP_COMMAND             0x314U
#define CAN_ID_H723_TURN_OVERRIDE                0x315U
#define CAN_ID_H723_TURN_MODE                    0x316U

#define FRONT_COMMAND_TX_PERIOD_MS               100U
#define FRONT_ACTUATOR_TARGET_MAX               4095U

/**
 * @brief Ethernet DMA와 LwIP 메모리가 위치하는 RAM_D2 전체를
 *        Normal, Shareable, Non-cacheable 영역으로 설정한다.
 *
 * RAM_D2:
 *   0x30000000 ~ 0x30007FFF (32 KB)
 *
 * CubeMX가 생성한 MPU Region 0은 그대로 두고 Region 1만 추가한다.
 */
static void EthernetMPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x30000000U;
    MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
    MPU_InitStruct.SubRegionDisable = 0x00U;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
 * @brief uint32_t 값을 Little Endian 형식으로 4바이트에 저장
 */
static void CAN_WriteU32LE(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

/**
 * @brief Little Endian 형식의 4바이트를 uint32_t로 변환
 */
static uint32_t CAN_ReadU32LE(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
 * @brief FDCAN2 Classical CAN 250 kbps 시작
 *
 * 송신:
 *   Standard ID 0x201
 *   A5 5A 02 01 + 32-bit counter
 *
 * 수신:
 *   Standard ID 0x281
 *   5A A5 02 02 + same counter
 */
static void CAN250_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* Filter 0: all split F407 status IDs from 0x210 through 0x21E. */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_ID_F407_STEERING_ADC;
    filter.FilterID2 = CAN_ID_F407_CAN_BUSOFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* Filter 1: exact 0x281 ping response. */
    filter.FilterIndex = 1U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterID1 = CAN_ID_F407_ECHO;
    filter.FilterID2 = 0x7FFU;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan2,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(
            &hfdcan2,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    can250_tx_header.Identifier = 0x201;
    can250_tx_header.IdType = FDCAN_STANDARD_ID;
    can250_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    can250_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    can250_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can250_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    can250_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    can250_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    can250_tx_header.MessageMarker = 0U;

}

/**
 * @brief FDCAN2 250 kbps Ping 송신
 */
static void CAN250_SendPing(void)
{
    uint32_t counter;

    can250_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    if (can250_tx_fifo_free == 0U)
    {
        can250_tx_busy_count++;
        can250_last_error = HAL_FDCAN_GetError(&hfdcan2);
        return;
    }

    counter = can250_next_counter++;

    can250_tx_data[0] = 0xA5;
    can250_tx_data[1] = 0x5A;
    can250_tx_data[2] = 0x02;
    can250_tx_data[3] = 0x01;
    CAN_WriteU32LE(&can250_tx_data[4], counter);

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan2,
            &can250_tx_header,
            can250_tx_data) == HAL_OK)
    {
        can250_last_sent_counter = counter;
        can250_tx_count++;
    }
    else
    {
        can250_tx_error_count++;
        can250_last_error = HAL_FDCAN_GetError(&hfdcan2);
    }
}

static HAL_StatusTypeDef FrontCommand_SendRaw(uint32_t identifier,
                                                    const uint8_t data[8],
                                                    uint32_t data_length)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        front_command_tx_busy_count++;
        front_command_last_error = HAL_FDCAN_GetError(&hfdcan2);
        return HAL_BUSY;
    }

    header.Identifier = identifier & 0x7FFU;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = data_length;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, (uint8_t *)data) != HAL_OK)
    {
        front_command_tx_error_count++;
        front_command_last_error = HAL_FDCAN_GetError(&hfdcan2);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef FrontCommand_SendU8(uint32_t identifier, uint8_t value)
{
    uint8_t data[8] = {0U};
    data[0] = value;
    return FrontCommand_SendRaw(identifier, data, FDCAN_DLC_BYTES_1);
}

static HAL_StatusTypeDef FrontCommand_SendU16(uint32_t identifier, uint16_t value)
{
    uint8_t data[8] = {0U};
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    return FrontCommand_SendRaw(identifier, data, FDCAN_DLC_BYTES_2);
}

static void FrontCommand_SchedulerInit(uint32_t now)
{
    front_steering_override_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_actuator_enable_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_actuator_target_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_headlamp_override_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_headlamp_command_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_turn_override_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_turn_mode_last_tx_tick = now - FRONT_COMMAND_TX_PERIOD_MS;
    front_command_scheduler_index = 0U;
}

static void FrontCommand_SendNext(uint32_t now)
{
    uint8_t attempt;
    uint16_t target;
    HAL_StatusTypeDef status;

    for (attempt = 0U; attempt < 7U; attempt++)
    {
        status = HAL_ERROR;

        switch (front_command_scheduler_index)
        {
            case 0U:
                if ((uint32_t)(now - front_steering_override_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_STEERING_OVERRIDE, front_steering_override_enable);
                    if (status == HAL_OK) { front_steering_override_last_tx_tick = now; front_steering_override_tx_count++; }
                }
                break;
            case 1U:
                if ((uint32_t)(now - front_actuator_enable_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_ACTUATOR_ENABLE, front_actuator_enable_command);
                    if (status == HAL_OK) { front_actuator_enable_last_tx_tick = now; front_actuator_enable_tx_count++; }
                }
                break;
            case 2U:
                if ((uint32_t)(now - front_actuator_target_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    target = front_actuator_target_command;
                    if (target > FRONT_ACTUATOR_TARGET_MAX) target = FRONT_ACTUATOR_TARGET_MAX;
                    status = FrontCommand_SendU16(CAN_ID_H723_ACTUATOR_TARGET, target);
                    if (status == HAL_OK) { front_actuator_target_last_tx_tick = now; front_actuator_target_tx_count++; }
                }
                break;
            case 3U:
                if ((uint32_t)(now - front_headlamp_override_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_HEADLAMP_OVERRIDE, front_headlamp_override_enable);
                    if (status == HAL_OK) { front_headlamp_override_last_tx_tick = now; front_headlamp_override_tx_count++; }
                }
                break;
            case 4U:
                if ((uint32_t)(now - front_headlamp_command_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_HEADLAMP_COMMAND, front_headlamp_command);
                    if (status == HAL_OK) { front_headlamp_command_last_tx_tick = now; front_headlamp_command_tx_count++; }
                }
                break;
            case 5U:
                if ((uint32_t)(now - front_turn_override_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_TURN_OVERRIDE, front_turn_override_enable);
                    if (status == HAL_OK) { front_turn_override_last_tx_tick = now; front_turn_override_tx_count++; }
                }
                break;
            default:
                if ((uint32_t)(now - front_turn_mode_last_tx_tick) >= FRONT_COMMAND_TX_PERIOD_MS)
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_TURN_MODE,
                                                 (front_turn_mode_command <= 2U) ? front_turn_mode_command : 0U);
                    if (status == HAL_OK) { front_turn_mode_last_tx_tick = now; front_turn_mode_tx_count++; }
                }
                break;
        }

        front_command_scheduler_index++;
        if (front_command_scheduler_index >= 7U) front_command_scheduler_index = 0U;
        if ((status == HAL_OK) || (status == HAL_BUSY)) return;
    }
}

/**
 * @brief FDCAN2 hardware state를 디버거용 변수에 갱신
 */
static void CAN_UpdateDiagnostics(void)
{
    can250_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    if (HAL_FDCAN_GetErrorCounters(
            &hfdcan2,
            (FDCAN_ErrorCountersTypeDef *)&can250_error_counters) != HAL_OK)
    {
        can250_last_error = HAL_FDCAN_GetError(&hfdcan2);
    }

    if (HAL_FDCAN_GetProtocolStatus(
            &hfdcan2,
            (FDCAN_ProtocolStatusTypeDef *)&can250_protocol_status) != HAL_OK)
    {
        can250_last_error = HAL_FDCAN_GetError(&hfdcan2);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  EthernetMPU_Config();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  CAN250_Start();

  {
      uint32_t start_tick = HAL_GetTick();

      /* Send the first 0x201 ping immediately after startup. */
      can250_last_tx_tick = start_tick - CAN250_TX_PERIOD_MS;

      FrontCommand_SchedulerInit(start_tick);

      if (FrontZoneNetwork_Init() != ERR_OK)
      {
          Error_Handler();
      }

      /* Telemetry stream toward the Jetson (/dev/vehicle_status). */
      if (FrontTelemetry_Init() != ERR_OK)
      {
          Error_Handler();
      }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      CAN_UpdateDiagnostics();


      if ((now - can250_last_tx_tick) >= CAN250_TX_PERIOD_MS)
      {
          can250_last_tx_tick = now;
          CAN250_SendPing();
      }

      FrontCommand_SendNext(now);

      FrontZoneNetwork_Process(now);

      /* Standalone(NO_SYS=1) LwIP packet, timeout, link processing */
      MX_LWIP_Process();

      /* Publish the parsed CAN state table to the Jetson (10 Hz). */
      {
          struct ft_values v;

          v.steering_adc = f407_steering_adc;
          v.accel_adc    = f407_accel_adc;
          v.brake_adc    = f407_brake_adc;
          v.actuator_pos = f407_actuator_position;
          v.temp_x10     = f407_temperature_x10;
          v.humidity_x10 = f407_humidity_x10;
          v.dht_valid    = f407_dht11_valid;
          v.turn_left    = f407_turn_left_output;
          v.turn_right   = f407_turn_right_output;
          v.headlamp     = f407_headlamp_output;
          v.busoff       = f407_can_busoff;

          FrontTelemetry_Process(&v);
      }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief FDCAN RX FIFO0 새 메시지 수신 콜백
 */
void HAL_FDCAN_RxFifo0Callback(
    FDCAN_HandleTypeDef *hfdcan,
    uint32_t RxFifo0ITs)
{
    if (((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) ||
        (hfdcan->Instance != FDCAN2))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        if (HAL_FDCAN_GetRxMessage(
                hfdcan,
                FDCAN_RX_FIFO0,
                &can250_rx_header,
                can250_rx_data) != HAL_OK)
        {
            can250_rx_bad_count++;
            can250_last_error = HAL_FDCAN_GetError(hfdcan);
            return;
        }

        can250_rx_count++;
        front_last_rx_id = can250_rx_header.Identifier;

        if ((can250_rx_header.IdType != FDCAN_STANDARD_ID) ||
            (can250_rx_header.RxFrameType != FDCAN_DATA_FRAME))
        {
            can250_rx_bad_count++;
            continue;
        }

        switch (can250_rx_header.Identifier)
        {
            case CAN_ID_F407_STEERING_ADC:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_steering_adc =
                        (uint16_t)((uint16_t)can250_rx_data[0] |
                                   ((uint16_t)can250_rx_data[1] << 8U));
                    f407_steering_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_BRAKE_ADC:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_brake_adc =
                        (uint16_t)((uint16_t)can250_rx_data[0] |
                                   ((uint16_t)can250_rx_data[1] << 8U));
                    f407_brake_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_ACCEL_ADC:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_accel_adc =
                        (uint16_t)((uint16_t)can250_rx_data[0] |
                                   ((uint16_t)can250_rx_data[1] << 8U));
                    f407_accel_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_TURN_LEFT_OUTPUT:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_turn_left_output = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_turn_left_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_TURN_RIGHT_OUTPUT:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_turn_right_output = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_turn_right_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_TURN_SWITCH_PRESSED:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_turn_switch_pressed = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_turn_switch_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_HEADLAMP_OUTPUT:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_headlamp_output = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_headlamp_output_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_TEMPERATURE_X10:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_temperature_x10 =
                        (int16_t)((uint16_t)can250_rx_data[0] |
                                  ((uint16_t)can250_rx_data[1] << 8U));
                    f407_temperature_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_HUMIDITY_X10:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_humidity_x10 =
                        (uint16_t)((uint16_t)can250_rx_data[0] |
                                   ((uint16_t)can250_rx_data[1] << 8U));
                    f407_humidity_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_DHT11_VALID:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_dht11_valid = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_dht11_valid_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_ACTUATOR_POSITION:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_2)
                {
                    f407_actuator_position =
                        (uint16_t)((uint16_t)can250_rx_data[0] |
                                   ((uint16_t)can250_rx_data[1] << 8U));
                    f407_actuator_position_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_ACTUATOR_ENABLE:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_actuator_output_enable = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_actuator_enable_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_STEERING_OVERRIDE_ACTIVE:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_steering_override_active = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_steering_override_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_HEADLAMP_OVERRIDE_ACTIVE:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_headlamp_override_active = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_headlamp_override_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_CAN_BUSOFF:
                if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_1)
                {
                    f407_can_busoff = (can250_rx_data[0] != 0U) ? 1U : 0U;
                    f407_busoff_rx_count++;
                }
                else can250_rx_bad_count++;
                break;

            case CAN_ID_F407_ECHO:
                if ((can250_rx_header.DataLength == FDCAN_DLC_BYTES_8) &&
                    (can250_rx_data[0] == 0x5AU) &&
                    (can250_rx_data[1] == 0xA5U) &&
                    (can250_rx_data[2] == 0x02U) &&
                    (can250_rx_data[3] == 0x02U))
                {
                    can250_last_received_counter = CAN_ReadU32LE(&can250_rx_data[4]);
                    if (can250_last_received_counter == can250_last_sent_counter)
                    {
                        can250_rx_ok_count++;
                    }
                    else
                    {
                        can250_rx_bad_count++;
                    }
                }
                else
                {
                    can250_rx_bad_count++;
                }
                break;

            default:
                front_unexpected_rx_count++;
                break;
        }
    }
}

/**
 * @brief FDCAN 오류 콜백
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN2)
    {
        can250_last_error = HAL_FDCAN_GetError(hfdcan);
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
