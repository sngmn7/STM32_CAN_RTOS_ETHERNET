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
#include "rear_zone_network.h"
#include "rear_telemetry.h"
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
/* FDCAN1: Classical CAN 500 kbps                                              */
/* H723 -> F446/PCAN: ID 0x101                                                 */
/* F446 echo response expected: ID 0x181                                       */
/* -------------------------------------------------------------------------- */
static FDCAN_TxHeaderTypeDef can500_tx_header;
static FDCAN_RxHeaderTypeDef can500_rx_header;
static uint8_t can500_tx_data[8];
static uint8_t can500_rx_data[8];

volatile uint32_t can500_next_counter = 0;
volatile uint32_t can500_last_sent_counter = 0;
volatile uint32_t can500_last_received_counter = 0;

volatile uint32_t can500_tx_count = 0;
volatile uint32_t can500_rx_count = 0;
volatile uint32_t can500_rx_ok_count = 0;
volatile uint32_t can500_rx_bad_count = 0;
volatile uint32_t can500_tx_busy_count = 0;
volatile uint32_t can500_tx_error_count = 0;
volatile uint32_t can500_last_error = 0;
volatile uint32_t can500_tx_fifo_free = 0;

volatile FDCAN_ErrorCountersTypeDef can500_error_counters;
volatile FDCAN_ProtocolStatusTypeDef can500_protocol_status;

static uint32_t can500_last_tx_tick = 0;

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

/* -------------------------------------------------------------------------- */
/* F446RE -> H723 센서 값 (FDCAN2 250 kbps, ID 0x301 / 0x311 / 0x312)          */
/* -------------------------------------------------------------------------- */
volatile uint8_t  f446_imu_data[8] = {0};   /* BNO085 rotation vector 원본 */
volatile uint8_t  f446_switch_pressed = 0;  /* 0 = 안눌림, 1 = 눌림        */
volatile uint16_t f446_ultrasonic_mm = 0;   /* 초음파 거리 (mm)            */

volatile uint32_t f446_imu_rx_count = 0;
volatile uint32_t f446_switch_rx_count = 0;
volatile uint32_t f446_ultrasonic_rx_count = 0;

/* 신선도(1초) 판정용 마지막 수신 시각 */
volatile uint32_t f446_imu_last_tick = 0;
volatile uint32_t f446_switch_last_tick = 0;
volatile uint32_t f446_ultrasonic_last_tick = 0;

/* -------------------------------------------------------------------------- */
/* H723 -> F446RE LED command over FDCAN2                                      */
/* Standard ID 0x310                                                          */
/* DATA[0] = left command, DATA[1] = right command, DATA[2] = brake command    */
/* -------------------------------------------------------------------------- */
static FDCAN_TxHeaderTypeDef led_cmd_tx_header;
static uint8_t led_cmd_tx_data[8];

static FDCAN_TxHeaderTypeDef window_cmd_tx_header;
static uint8_t window_cmd_tx_data[8];

volatile uint8_t led_auto_test_enable = 0U;
volatile uint8_t led_test_step = 0U;
volatile uint8_t led_left_command = 0U;
volatile uint8_t led_right_command = 0U;
volatile uint8_t led_brake_command = 0U;
volatile uint8_t window_command = 0U;

volatile uint32_t led_cmd_tx_count = 0U;
volatile uint32_t led_cmd_tx_busy_count = 0U;
volatile uint32_t led_cmd_tx_error_count = 0U;
volatile uint32_t led_cmd_last_error = 0U;

volatile uint32_t window_cmd_tx_count = 0U;
volatile uint32_t window_cmd_tx_busy_count = 0U;
volatile uint32_t window_cmd_tx_error_count = 0U;
volatile uint32_t window_cmd_last_error = 0U;

static uint32_t led_cmd_last_tx_tick = 0U;
static uint32_t led_test_last_step_tick = 0U;
static uint32_t window_cmd_last_tx_tick = 0U;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void CAN500_Start(void);
static void CAN500_SendPing(void);

static void CAN250_Start(void);
static void CAN250_SendPing(void);

static void LED_CommandSetStep(uint8_t step);
static void LED_CommandSend(void);
static void LED_CommandUpdate(void);

static void Window_CommandSend(void);
static void Window_CommandUpdate(void);
static void UDP_ApplyPendingCommand(void);

static void CAN_UpdateDiagnostics(void);

static void CAN_WriteU32LE(uint8_t *data, uint32_t value);
static uint32_t CAN_ReadU32LE(const uint8_t *data);
static void EthernetMPU_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CAN500_TX_PERIOD_MS       500U
#define CAN250_TX_PERIOD_MS       500U
#define CAN_TX_PHASE_OFFSET_MS    250U

#define REAR_LEFT_LAMP_CMD_ID     0x320U
#define REAR_RIGHT_LAMP_CMD_ID    0x321U
#define REAR_BRAKE_LAMP_CMD_ID    0x322U
#define WINDOW_CMD_ID             0x323U

/* F446RE -> H723 센서 신호 (FDCAN2 250 kbps) */
#define CAN_ID_F446_IMU           0x301U   /* DLC 8 : BNO085 rotation vector */
#define CAN_ID_F446_SWITCH        0x311U   /* DLC 1 : 토글스위치 0/1        */
#define CAN_ID_F446_ULTRASONIC    0x312U   /* DLC 2 : 거리 u16 LE (mm)      */
#define WINDOW_CMD_TX_PERIOD_MS   100U
#define LED_CMD_TX_PERIOD_MS      100U
#define LED_TEST_STEP_PERIOD_MS   3000U
#define LED_TEST_STEP_COUNT       5U


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
 * @brief FDCAN1 Classical CAN 500 kbps 시작
 *
 * 송신:
 *   Standard ID 0x101
 *   A5 5A 01 01 + 32-bit counter
 *
 * 수신:
 *   Standard ID 0x181
 *   5A A5 01 02 + same counter
 */
static void CAN500_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x181;
    filter.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_REJECT_REMOTE,
            FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    can500_tx_header.Identifier = 0x101;
    can500_tx_header.IdType = FDCAN_STANDARD_ID;
    can500_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    can500_tx_header.DataLength = FDCAN_DLC_BYTES_8;
    can500_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can500_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    can500_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    can500_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    can500_tx_header.MessageMarker = 0U;
}

/**
 * @brief FDCAN1 500 kbps Ping 송신
 */
static void CAN500_SendPing(void)
{
    uint32_t counter;

    can500_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

    if (can500_tx_fifo_free == 0U)
    {
        can500_tx_busy_count++;
        can500_last_error = HAL_FDCAN_GetError(&hfdcan1);
        return;
    }

    counter = can500_next_counter++;

    can500_tx_data[0] = 0xA5;
    can500_tx_data[1] = 0x5A;
    can500_tx_data[2] = 0x01;
    can500_tx_data[3] = 0x01;
    CAN_WriteU32LE(&can500_tx_data[4], counter);

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1,
            &can500_tx_header,
            can500_tx_data) == HAL_OK)
    {
        can500_last_sent_counter = counter;
        can500_tx_count++;
    }
    else
    {
        can500_tx_error_count++;
        can500_last_error = HAL_FDCAN_GetError(&hfdcan1);
    }
}

/**
 * @brief FDCAN2 Classical CAN 250 kbps 시작
 *
 * 송신:
 *   Standard ID 0x201
 *   A5 5A 02 01 + 32-bit counter
 *
 * 수신:
 *   Standard ID 0x281        핑 에코 응답
 *   5A A5 02 02 + same counter
 *
 *   Standard ID 0x301~0x312  F446RE 센서 신호
 *     0x301 IMU(BNO085) / 0x311 토글스위치 / 0x312 초음파 거리(mm)
 */
static void CAN250_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* 필터0: 핑 에코 응답 0x281 단일 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x281;
    filter.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /*
     * 필터1: F446RE 센서 신호 0x301~0x312 범위.
     * 글로벌 필터가 REJECT이므로 이 필터가 없으면 IMU/스위치/초음파가
     * RxFIFO에 들어오지 못한다.
     */
    filter.FilterIndex = 1;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_ID_F446_IMU;
    filter.FilterID2 = CAN_ID_F446_ULTRASONIC;

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

    /* H723 -> F446RE LED 명령 프레임 */
    led_cmd_tx_header.Identifier = REAR_LEFT_LAMP_CMD_ID;
    led_cmd_tx_header.IdType = FDCAN_STANDARD_ID;
    led_cmd_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    led_cmd_tx_header.DataLength = FDCAN_DLC_BYTES_1;
    led_cmd_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    led_cmd_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    led_cmd_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    led_cmd_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    led_cmd_tx_header.MessageMarker = 0U;

    /* H723 -> window controller command frame */
    window_cmd_tx_header.Identifier = WINDOW_CMD_ID;
    window_cmd_tx_header.IdType = FDCAN_STANDARD_ID;
    window_cmd_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    window_cmd_tx_header.DataLength = FDCAN_DLC_BYTES_1;
    window_cmd_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    window_cmd_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    window_cmd_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    window_cmd_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    window_cmd_tx_header.MessageMarker = 0U;
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

/**
 * @brief 자동 LED 시험 단계에 맞춰 명령값 설정
 *
 * step 0: 모두 OFF
 * step 1: 왼쪽 방향지시등 활성화(F446RE에서 500 ms 점멸)
 * step 2: 오른쪽 방향지시등 활성화(F446RE에서 500 ms 점멸)
 * step 3: 브레이크등 ON
 * step 4: 왼쪽 + 오른쪽 + 브레이크 활성화
 */
static void LED_CommandSetStep(uint8_t step)
{
    led_test_step = (uint8_t)(step % LED_TEST_STEP_COUNT);

    switch (led_test_step)
    {
        case 0U:
            led_left_command = 0U;
            led_right_command = 0U;
            led_brake_command = 0U;
            break;

        case 1U:
            led_left_command = 1U;
            led_right_command = 0U;
            led_brake_command = 0U;
            break;

        case 2U:
            led_left_command = 0U;
            led_right_command = 1U;
            led_brake_command = 0U;
            break;

        case 3U:
            led_left_command = 0U;
            led_right_command = 0U;
            led_brake_command = 1U;
            break;

        default: /* step 4 */
            led_left_command = 1U;
            led_right_command = 1U;
            led_brake_command = 1U;
            break;
    }
}

/**
 * @brief LED 명령 0x310 송신
 *
 * byte 0: left  (0/1)
 * byte 1: right (0/1)
 * byte 2: brake (0/1)
 * byte 3: 현재 시험 step (디버깅용, F446RE에서는 무시)
 * byte 4~7: 0
 */
static void RearLamp_SendU8(uint32_t identifier, uint8_t value)
{
    uint8_t data[8] = {0U};

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        led_cmd_tx_busy_count++;
        led_cmd_last_error = HAL_FDCAN_GetError(&hfdcan2);
        return;
    }

    led_cmd_tx_header.Identifier = identifier;
    led_cmd_tx_header.DataLength = FDCAN_DLC_BYTES_1;
    data[0] = (value != 0U) ? 1U : 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &led_cmd_tx_header, data) == HAL_OK)
    {
        led_cmd_tx_count++;
    }
    else
    {
        led_cmd_tx_error_count++;
        led_cmd_last_error = HAL_FDCAN_GetError(&hfdcan2);
    }
}

static void LED_CommandSend(void)
{
    RearLamp_SendU8(REAR_LEFT_LAMP_CMD_ID, led_left_command);
    RearLamp_SendU8(REAR_RIGHT_LAMP_CMD_ID, led_right_command);
    RearLamp_SendU8(REAR_BRAKE_LAMP_CMD_ID, led_brake_command);
}

/**
 * @brief LED 자동 시험 패턴 갱신 및 주기 송신
 *
 * led_auto_test_enable = 1: 3초마다 시험 단계 자동 변경
 * led_auto_test_enable = 0: Live Expressions에서 세 command 값을 수동 변경
 */
static void LED_CommandUpdate(void)
{
    uint32_t now = HAL_GetTick();

    if ((led_auto_test_enable != 0U) &&
        ((now - led_test_last_step_tick) >= LED_TEST_STEP_PERIOD_MS))
    {
        led_test_last_step_tick = now;
        LED_CommandSetStep((uint8_t)(led_test_step + 1U));
    }

    if ((now - led_cmd_last_tx_tick) >= LED_CMD_TX_PERIOD_MS)
    {
        led_cmd_last_tx_tick = now;
        LED_CommandSend();
    }
}

/**
 * @brief Window command 0x311 transmission.
 *
 * DATA[0] = window command (0=OFF, 1=ON)
 * DATA[1..7] = 0
 */
static void Window_CommandSend(void)
{
    window_cmd_tx_data[0] = (window_command != 0U) ? 1U : 0U;
    window_cmd_tx_data[1] = 0U;
    window_cmd_tx_data[2] = 0U;
    window_cmd_tx_data[3] = 0U;
    window_cmd_tx_data[4] = 0U;
    window_cmd_tx_data[5] = 0U;
    window_cmd_tx_data[6] = 0U;
    window_cmd_tx_data[7] = 0U;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        window_cmd_tx_busy_count++;
        window_cmd_last_error = HAL_FDCAN_GetError(&hfdcan2);
        return;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan2,
            &window_cmd_tx_header,
            window_cmd_tx_data) == HAL_OK)
    {
        window_cmd_tx_count++;
    }
    else
    {
        window_cmd_tx_error_count++;
        window_cmd_last_error = HAL_FDCAN_GetError(&hfdcan2);
    }
}


/**
 * @brief Periodically repeat the current window command.
 */
static void Window_CommandUpdate(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - window_cmd_last_tx_tick) >= WINDOW_CMD_TX_PERIOD_MS)
    {
        window_cmd_last_tx_tick = now;
        Window_CommandSend();
    }
}


/**
 * @brief Apply one pending 4-byte UDP command to the CAN actuator commands.
 */
static void UDP_ApplyPendingCommand(void)
{
    RearZoneNetwork_Process(HAL_GetTick());
}


/**
 * @brief 두 FDCAN 인스턴스의 하드웨어 상태를 디버거용 변수에 갱신
 */
static void CAN_UpdateDiagnostics(void)
{
    can500_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    can250_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);

    if (HAL_FDCAN_GetErrorCounters(
            &hfdcan1,
            (FDCAN_ErrorCountersTypeDef *)&can500_error_counters) != HAL_OK)
    {
        can500_last_error = HAL_FDCAN_GetError(&hfdcan1);
    }

    if (HAL_FDCAN_GetProtocolStatus(
            &hfdcan1,
            (FDCAN_ProtocolStatusTypeDef *)&can500_protocol_status) != HAL_OK)
    {
        can500_last_error = HAL_FDCAN_GetError(&hfdcan1);
    }

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  /* Jetson command server (5001) + Front/Rear inter-zone server (5003). */
  if (RearZoneNetwork_Init() != ERR_OK)
  {
      Error_Handler();
  }

  /* Telemetry stream toward the Jetson (/dev/rear_status). */
  if (RearTelemetry_Init() != ERR_OK)
  {
      Error_Handler();
  }

  CAN500_Start();
  CAN250_Start();
  {
      uint32_t start_tick = HAL_GetTick();

      /*
       * 두 버스 모두 500 ms 주기이지만 250 ms 위상차를 둔다.
       * 동시에 실행해도 문제는 없지만 PCAN에서 구분하기 쉽게 하기 위함이다.
       */
      can500_last_tx_tick = start_tick;
      can250_last_tx_tick = start_tick - CAN_TX_PHASE_OFFSET_MS;

      /* Start all actuators in the OFF state. */
      LED_CommandSetStep(0U);
      window_command = 0U;

      led_test_last_step_tick = start_tick;
      led_cmd_last_tx_tick = start_tick - LED_CMD_TX_PERIOD_MS;
      window_cmd_last_tx_tick = start_tick - WINDOW_CMD_TX_PERIOD_MS;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      CAN_UpdateDiagnostics();

      if ((now - can500_last_tx_tick) >= CAN500_TX_PERIOD_MS)
      {
          can500_last_tx_tick = now;
          CAN500_SendPing();
      }

      if ((now - can250_last_tx_tick) >= CAN250_TX_PERIOD_MS)
      {
          can250_last_tx_tick = now;
          CAN250_SendPing();
      }

      /* Standalone(NO_SYS=1) LwIP packet, timeout, link processing */
      MX_LWIP_Process();

      /* Apply a newly received Jetson command, if any. */
      UDP_ApplyPendingCommand();

      /* Repeat the latest actuator states over FDCAN2. */
      LED_CommandUpdate();
      Window_CommandUpdate();

      /* Publish the rear zone state table to the Jetson (10 Hz). */
      {
          struct rt_values v;

          v.turn_mode         = rear_applied_turn_mode;
          v.led_left          = led_left_command;
          v.led_right         = led_right_command;
          v.led_brake         = led_brake_command;
          v.window            = window_command;
          v.front_link_alive  = rear_front_link_alive;
          v.jetson_link_alive = rear_jetson_command_alive;
          v.turn_src_jetson   = rear_turn_source_jetson;
          v.brake_src_jetson  = rear_brake_source_jetson;
          v.can500_error      = can500_protocol_status.BusOff ? 1U : 0U;
          v.can250_error      = can250_protocol_status.BusOff ? 1U : 0U;
          v.front_brake_adc   = rear_front_brake_adc;
          v.can500_rx_count   = can500_rx_count;
          v.can250_rx_count   = can250_rx_count;
          v.cmd_rx_count      = rear_jetson_command_rx_count;

          /* F466RE 에서 CAN 으로 올라온 센서 (v2) */
          v.ultrasonic_mm     = f446_ultrasonic_mm;
          v.imu_raw           = (const uint8_t *)f446_imu_data;
          v.switch_pressed    = f446_switch_pressed;

          v.ultra_last_tick   = f446_ultrasonic_last_tick;
          v.imu_last_tick     = f446_imu_last_tick;
          v.switch_last_tick  = f446_switch_last_tick;

          v.imu_rx_count      = f446_imu_rx_count;
          v.ultra_rx_count    = f446_ultrasonic_rx_count;
          v.switch_rx_count   = f446_switch_rx_count;

          RearTelemetry_Process(&v);
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
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    if (hfdcan->Instance == FDCAN1)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(
                   hfdcan,
                   FDCAN_RX_FIFO0) > 0U)
        {
            if (HAL_FDCAN_GetRxMessage(
                    hfdcan,
                    FDCAN_RX_FIFO0,
                    &can500_rx_header,
                    can500_rx_data) != HAL_OK)
            {
                can500_rx_bad_count++;
                can500_last_error = HAL_FDCAN_GetError(hfdcan);
                return;
            }

            can500_rx_count++;

            if ((can500_rx_header.IdType == FDCAN_STANDARD_ID) &&
                (can500_rx_header.Identifier == 0x181) &&
                (can500_rx_header.RxFrameType == FDCAN_DATA_FRAME) &&
                (can500_rx_header.DataLength == FDCAN_DLC_BYTES_8) &&
                (can500_rx_data[0] == 0x5A) &&
                (can500_rx_data[1] == 0xA5) &&
                (can500_rx_data[2] == 0x01) &&
                (can500_rx_data[3] == 0x02))
            {
                can500_last_received_counter =
                    CAN_ReadU32LE(&can500_rx_data[4]);

                if (can500_last_received_counter ==
                    can500_last_sent_counter)
                {
                    can500_rx_ok_count++;
                }
                else
                {
                    can500_rx_bad_count++;
                }
            }
            else
            {
                can500_rx_bad_count++;
            }
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(
                   hfdcan,
                   FDCAN_RX_FIFO0) > 0U)
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

            if ((can250_rx_header.IdType == FDCAN_STANDARD_ID) &&
                (can250_rx_header.Identifier == 0x281) &&
                (can250_rx_header.RxFrameType == FDCAN_DATA_FRAME) &&
                (can250_rx_header.DataLength == FDCAN_DLC_BYTES_8) &&
                (can250_rx_data[0] == 0x5A) &&
                (can250_rx_data[1] == 0xA5) &&
                (can250_rx_data[2] == 0x02) &&
                (can250_rx_data[3] == 0x02))
            {
                can250_last_received_counter =
                    CAN_ReadU32LE(&can250_rx_data[4]);

                if (can250_last_received_counter ==
                    can250_last_sent_counter)
                {
                    can250_rx_ok_count++;
                }
                else
                {
                    can250_rx_bad_count++;
                }
            }
            else if (can250_rx_header.IdType == FDCAN_STANDARD_ID)
            {
                switch (can250_rx_header.Identifier)
                {
                    case CAN_ID_F446_IMU:
                        if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_8)
                        {
                            uint8_t i;
                            for (i = 0U; i < 8U; i++)
                            {
                                f446_imu_data[i] = can250_rx_data[i];
                            }
                            f446_imu_last_tick = HAL_GetTick();
                            f446_imu_rx_count++;
                        }
                        break;

                    case CAN_ID_F446_SWITCH:
                        /* F446RE는 DLC 8 고정 송신, data[0]만 유효(나머지 0 패딩) */
                        if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_8)
                        {
                            f446_switch_pressed =
                                (can250_rx_data[0] != 0U) ? 1U : 0U;
                            f446_switch_last_tick = HAL_GetTick();
                            f446_switch_rx_count++;
                        }
                        break;

                    case CAN_ID_F446_ULTRASONIC:
                        /* F446RE는 DLC 8 고정 송신, data[0..1]만 유효 */
                        if (can250_rx_header.DataLength == FDCAN_DLC_BYTES_8)
                        {
                            f446_ultrasonic_mm =
                                (uint16_t)((uint16_t)can250_rx_data[0] |
                                           ((uint16_t)can250_rx_data[1] << 8U));
                            f446_ultrasonic_last_tick = HAL_GetTick();
                            f446_ultrasonic_rx_count++;
                        }
                        break;

                    default:
                        can250_rx_bad_count++;
                        break;
                }
            }
            else
            {
                can250_rx_bad_count++;
            }
        }
    }
}

/**
 * @brief FDCAN 오류 콜백
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        can500_last_error = HAL_FDCAN_GetError(hfdcan);
    }
    else if (hfdcan->Instance == FDCAN2)
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
  /* User can add an error indication here if needed. */
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
