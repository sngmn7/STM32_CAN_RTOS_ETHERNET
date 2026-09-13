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
#include "perf.h"
#include "diagnostic.h"
#include "eth_phy_recover.h"

/*
 * 계측/진단 마스터 스위치 — HardFault 원인 이등분용.
 * 0 = perf(DWT) / diagnostic(DTC) 전부 비활성
 */
#define FRONT_INSTRUMENT_ENABLE   1
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
/* Downstream command frames accepted by the FDCAN2 TX FIFO. Paired with
   F407_Front's can_command_rx_count to measure link loss exactly. */
volatile uint32_t front_command_tx_count = 0U;

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
static void CAN_BusOffRecover(uint32_t now);

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
/* 변화 시 즉시 전송하되 같은 신호의 최소 송신 간격은 지킨다. */
#define FRONT_COMMAND_MIN_GAP_MS                  10U
#define FRONT_ACTUATOR_TARGET_MAX               4095U

/* 계측: 가장 최근 CAN 프레임이 도착한 사이클 (ISR 에서 갱신) */
volatile uint32_t perf_can_stamp = 0U;
volatile uint8_t  perf_can_stamp_valid = 0U;

/* ------------------------------------------------------------------ */
/* 진단 (DTC)                                                          */
/*                                                                     */
/* confirm_ms 는 "정상 주기 x 3배 이상" 원칙을 따른다 (COMM_ICD.md §4). */
/* heal_ms 는 confirm_ms 보다 길게 두어 채터링을 막는다(히스테리시스).  */
/* ------------------------------------------------------------------ */

enum {
    DTC_IDX_ZONE_LINK = 0,   /* Front <-> Rear 링크 상실       */
    DTC_IDX_HOST_LINK,       /* Jetson 명령 링크 상실          */
    DTC_IDX_NODE_BUSOFF,     /* F407 이 버스오프를 보고        */
    DTC_IDX_NODE_SIGNAL,     /* F407 CAN 신호 두절             */
    DTC_IDX_DHT_INVALID,     /* DHT11 온습도 무효              */
    DTC_IDX_COUNT
};

/*
 * 진단 보정값(calibration). const 를 빼 RAM 에 두는 이유는 두 가지다.
 *
 *  1) confirm_ms / heal_ms 는 검출 시간과 오검출률의 트레이드오프를 정하는
 *     값이라 차량마다 재조정 대상이다. 양산 ECU 가 진단 보정값을 EEPROM 에
 *     두는 것과 같은 취지.
 *  2) 값을 바꿀 때마다 재빌드·재플래시하면 스윕 시험을 돌릴 수 없다.
 *     RAM 에 있으면 디버거로 써 넣고 바로 다음 조건을 측정할 수 있다.
 *
 * Diag_Init() 은 이 배열을 복사하지 않고 포인터로 참조하므로, 여기를 고치면
 * 다음 판정부터 즉시 반영된다.
 */
static diag_def_t front_dtc_defs[DTC_IDX_COUNT] = {
    /* code                       confirm  heal  warning */
    { DTC_U0100_ZONE_LINK_LOST,      1000U, 2000U, 1U },  /* 후방등 불능 → 경고 */
    { DTC_U0300_HOST_LINK_LOST,      1500U, 3000U, 0U },  /* 로컬 제어로 복귀    */
    { DTC_U0003_NODE_BUSOFF,          500U, 3000U, 1U },
    { DTC_P0510_NODE_SIGNAL_LOST,    1000U, 2000U, 1U },  /* 브레이크 신호 상실  */
    { DTC_P0503_DHT_INVALID,         5000U, 5000U, 0U },  /* 편의 기능, 경고 없음 */
};

/* CAN 신호 두절 판정용 — 브레이크 수신 카운터 정체 감시 */
static uint32_t dtc_brake_last_count = 0U;
static uint32_t dtc_brake_last_change_ms = 0U;

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
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    /*
     * Normal, Non-cacheable, Shareable (TEX=1, C=0, B=0).
     *
     * 참고 — 폴트 디버깅 시 Strongly-ordered(TEX=0)로 바꾸면
     * 쓰기 버퍼링이 없어져 IMPRECISERR 이 PRECISERR + BFAR 유효로
     * 정밀해지지만, lwIP 가 패킷 헤더에 비정렬 쓰기를 하므로
     * UNALIGNED UsageFault 가 대신 발생해 이 영역에는 쓸 수 없다.
     * (실측 확인함)
     */
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;

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

    front_command_tx_count++;
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

/* ------------------------------------------------------------------ */
/* 하행 명령 반영 지연 계측                                            */
/*                                                                     */
/* 값이 바뀐 시각부터 그 신호가 실제로 CAN 으로 나간 시각까지를 잰다.  */
/* 순수 주기 전송이면 자기 슬롯을 기다리므로 0~100 ms 로 흩어진다.     */
/* ------------------------------------------------------------------ */
#define FC_SIG_COUNT   7U

/* min_us 는 UINT32_MAX 로 시작해야 첫 표본이 그대로 들어간다. */
volatile perf_stat_t perf_cmd_latency = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };

/*
 * 종단 반영 지연 — 제어 루프가 실제로 닫히는 시간.
 *
 *   Jetson 명령 적용 -> CAN 하행 -> F407 동작 -> CAN 상행(f407_headlamp_output)
 *   -> H723 상태표 갱신 -> SV 텔레메트리 송신
 *
 * 양끝의 UDP 구간(편도 약 0.12 ms)만 빠지는데, ACK RTT 로 따로 쟀고
 * 전체 대비 무시할 만하다.
 */
volatile perf_stat_t perf_reflect_latency = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };

static uint8_t  refl_pending   = 0U;
static uint8_t  refl_target    = 0U;
static uint32_t refl_cycle     = 0U;
static uint16_t refl_last_cmd  = 0xFFFFU;   /* 첫 바퀴는 초기값 채우기만 */

static uint16_t fc_last_value[FC_SIG_COUNT];
static uint32_t fc_change_cycle[FC_SIG_COUNT];
static uint8_t  fc_pending[FC_SIG_COUNT];
static uint8_t  fc_tracker_primed = 0U;

static uint16_t FrontCommand_Value(uint8_t idx)
{
    switch (idx)
    {
        case 0U:  return (uint16_t)front_steering_override_enable;
        case 1U:  return (uint16_t)front_actuator_enable_command;
        case 2U:  return (uint16_t)front_actuator_target_command;
        case 3U:  return (uint16_t)front_headlamp_override_enable;
        case 4U:  return (uint16_t)front_headlamp_command;
        case 5U:  return (uint16_t)front_turn_override_enable;
        default:  return (uint16_t)front_turn_mode_command;
    }
}

static void FrontCommand_TrackChange(void)
{
    uint8_t i;
    uint32_t cyc = PERF_NOW();

    for (i = 0U; i < FC_SIG_COUNT; i++)
    {
        uint16_t v = FrontCommand_Value(i);
        if (v != fc_last_value[i])
        {
            fc_last_value[i] = v;
            if (fc_tracker_primed == 0U)
            {
                continue;    /* 첫 바퀴는 초기값 채우기만 */
            }
            if (fc_pending[i] == 0U)
            {
                fc_change_cycle[i] = cyc;
                fc_pending[i] = 1U;
            }
        }
    }
    fc_tracker_primed = 1U;
}

/**
 * @brief 이 신호를 지금 보내야 하는가
 *
 * 값이 바뀌어 대기 중이면 주기를 기다리지 않고 곧바로 내보낸다. 다만 같은
 * 신호가 최소 간격(FRONT_COMMAND_MIN_GAP_MS)보다 촘촘히 나가지는 않게 막아,
 * 값이 빠르게 흔들려도 버스를 채우지 않도록 한다.
 * 변화가 없으면 종전대로 주기 갱신(하트비트)만 한다.
 */
static uint8_t FrontCommand_Due(uint8_t idx, uint32_t now, uint32_t last_tx_tick)
{
    uint32_t age = (uint32_t)(now - last_tx_tick);

    if ((idx < FC_SIG_COUNT) && (fc_pending[idx] != 0U))
    {
        return (age >= FRONT_COMMAND_MIN_GAP_MS) ? 1U : 0U;
    }
    return (age >= FRONT_COMMAND_TX_PERIOD_MS) ? 1U : 0U;
}

static void FrontCommand_MarkSent(uint8_t idx)
{
    if ((idx >= FC_SIG_COUNT) || (fc_pending[idx] == 0U))
    {
        return;
    }
    fc_pending[idx] = 0U;
    Perf_Update(&perf_cmd_latency, PERF_ELAPSED(fc_change_cycle[idx]));
}

static void FrontCommand_SendNext(uint32_t now)
{
    uint8_t attempt;
    uint8_t idx;
    uint16_t target;
    HAL_StatusTypeDef status;

    FrontCommand_TrackChange();

    for (attempt = 0U; attempt < 7U; attempt++)
    {
        status = HAL_ERROR;
        idx = front_command_scheduler_index;

        switch (front_command_scheduler_index)
        {
            case 0U:
                if (FrontCommand_Due(0U, now, front_steering_override_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_STEERING_OVERRIDE, front_steering_override_enable);
                    if (status == HAL_OK) { front_steering_override_last_tx_tick = now; front_steering_override_tx_count++; }
                }
                break;
            case 1U:
                if (FrontCommand_Due(1U, now, front_actuator_enable_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_ACTUATOR_ENABLE, front_actuator_enable_command);
                    if (status == HAL_OK) { front_actuator_enable_last_tx_tick = now; front_actuator_enable_tx_count++; }
                }
                break;
            case 2U:
                if (FrontCommand_Due(2U, now, front_actuator_target_last_tx_tick))
                {
                    target = front_actuator_target_command;
                    if (target > FRONT_ACTUATOR_TARGET_MAX) target = FRONT_ACTUATOR_TARGET_MAX;
                    status = FrontCommand_SendU16(CAN_ID_H723_ACTUATOR_TARGET, target);
                    if (status == HAL_OK) { front_actuator_target_last_tx_tick = now; front_actuator_target_tx_count++; }
                }
                break;
            case 3U:
                if (FrontCommand_Due(3U, now, front_headlamp_override_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_HEADLAMP_OVERRIDE, front_headlamp_override_enable);
                    if (status == HAL_OK) { front_headlamp_override_last_tx_tick = now; front_headlamp_override_tx_count++; }
                }
                break;
            case 4U:
                if (FrontCommand_Due(4U, now, front_headlamp_command_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_HEADLAMP_COMMAND, front_headlamp_command);
                    if (status == HAL_OK) { front_headlamp_command_last_tx_tick = now; front_headlamp_command_tx_count++; }
                }
                break;
            case 5U:
                if (FrontCommand_Due(5U, now, front_turn_override_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_TURN_OVERRIDE, front_turn_override_enable);
                    if (status == HAL_OK) { front_turn_override_last_tx_tick = now; front_turn_override_tx_count++; }
                }
                break;
            default:
                if (FrontCommand_Due(6U, now, front_turn_mode_last_tx_tick))
                {
                    status = FrontCommand_SendU8(CAN_ID_H723_TURN_MODE,
                                                 (front_turn_mode_command <= 2U) ? front_turn_mode_command : 0U);
                    if (status == HAL_OK) { front_turn_mode_last_tx_tick = now; front_turn_mode_tx_count++; }
                }
                break;
        }

        if (status == HAL_OK) FrontCommand_MarkSent(idx);

        front_command_scheduler_index++;
        if (front_command_scheduler_index >= 7U) front_command_scheduler_index = 0U;
        if ((status == HAL_OK) || (status == HAL_BUSY)) return;
    }
}

/**
 * @brief 프리즈 프레임 캡처 — 최초 DTC 확정 시점의 신호 스냅샷
 *
 * 슬롯 의미는 COMM_ICD.md 에 정의한다 (Front 존 기준).
 * 사후 분석 시 "고장 당시 차량이 어떤 상태였나"를 재구성하는 용도.
 */
static void Front_CaptureFreeze(uint16_t *sig)
{
    sig[0] = f407_brake_adc;
    sig[1] = f407_steering_adc;
    sig[2] = f407_accel_adc;
    sig[3] = f407_actuator_position;
    sig[4] = (uint16_t)((f407_turn_left_output ? 1U : 0U) |
                        (f407_turn_right_output ? 2U : 0U) |
                        (f407_headlamp_output ? 4U : 0U) |
                        (f407_can_busoff ? 8U : 0U));
    sig[5] = (uint16_t)((front_rear_link_alive ? 1U : 0U) |
                        (front_jetson_command_alive ? 2U : 0U) |
                        (front_brake_pressed ? 4U : 0U) |
                        (front_brake_calibrated ? 8U : 0U));
    sig[6] = (uint16_t)(f407_brake_rx_count & 0xFFFFU);
    /* PHY 재시도 횟수 — 부팅 시 PHY 초기화가 몇 번 만에 성공했는지 */
    sig[7] = (uint16_t)(eth_phy_retry_count & 0xFFFFU);
}

/**
 * @brief 원시 고장 여부를 진단 모듈에 보고
 *
 * 판정만 하고 조치는 하지 않는다 — 페일세이프는 기존 로직이 담당하고,
 * 여기서는 "관측"만 한다. 판정과 조치를 분리해야 진단을 껐다 켜도
 * 차량 동작이 바뀌지 않는다.
 */
static void Front_ReportDiagnostics(uint32_t now)
{
    /* CAN 신호 두절: 브레이크 수신 카운터가 멈췄는가 */
    if (f407_brake_rx_count != dtc_brake_last_count)
    {
        dtc_brake_last_count = f407_brake_rx_count;
        dtc_brake_last_change_ms = now;
    }

    Diag_Report(DTC_IDX_ZONE_LINK,
                (front_rear_link_alive == 0U) ? 1U : 0U, now);

    Diag_Report(DTC_IDX_HOST_LINK,
                (front_jetson_command_alive == 0U) ? 1U : 0U, now);

    Diag_Report(DTC_IDX_NODE_BUSOFF,
                (f407_can_busoff != 0U) ? 1U : 0U, now);

    /* 브레이크는 20 ms 주기 → 300 ms 이상 정체면 두절로 본다 */
    Diag_Report(DTC_IDX_NODE_SIGNAL,
                ((uint32_t)(now - dtc_brake_last_change_ms) > 300U) ? 1U : 0U,
                now);

    Diag_Report(DTC_IDX_DHT_INVALID,
                (f407_dht11_valid == 0U) ? 1U : 0U, now);

    Diag_Process(now);
}

/**
 * @brief FDCAN2 hardware state를 디버거용 변수에 갱신
 */
/**
 * @brief FDCAN2 버스오프 자동 복구
 *
 * FDCAN 은 버스오프에 빠지면 하드웨어가 CCCR.INIT 를 세우고 그대로 멈춘다.
 * 소프트웨어가 INIT 를 내려야 표준 복구 시퀀스(129 x 11 recessive bit)가
 * 시작된다. HAL_FDCAN_Start() 는 초기화 때 한 번뿐이라 복구 경로가 없었다.
 *
 * Rear 에서 같은 결함을 고친 뒤 실제 버스오프에서 복구가 5회 동작하는 것을
 * 확인하고 Front 에도 이식했다.
 *
 * 재시도 간격을 두는 이유는, 배선이 끊긴 채로 두면 복구 -> 즉시 버스오프를
 * 반복하며 버스를 흔들기 때문이다.
 */
#define CAN_BUSOFF_RECOVER_MS   1000U

volatile uint32_t can250_busoff_recover_count = 0U;

static void CAN_BusOffRecover(uint32_t now)
{
    static uint32_t last_try = 0U;

    if (can250_protocol_status.BusOff == 0U)
    {
        return;
    }
    if ((uint32_t)(now - last_try) < CAN_BUSOFF_RECOVER_MS)
    {
        return;
    }
    last_try = now;
    CLEAR_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_INIT);
    can250_busoff_recover_count++;
}
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

  /*
   * Cortex-M7 D-Cache 전체 무효화 — 다른 어떤 초기화보다 먼저.
   *
   * 파워온 리셋 직후 캐시 RAM 에는 임의의 노이즈가 남아 있다.
   * Cortex-M7 캐시는 ECC 를 갖고 있어서, "전체 무효화"가 아닌
   * 범위 지정 연산(SCB_InvalidateDCache_by_Addr 등)을 수행하면
   * 노이즈가 든 라인에서 ECC 오류가 발생하고 BusFault/HardFault
   * (IMPRECISERR) 로 전파된다.
   *
   * 이더넷 수신 경로의 HAL_ETH_RxLinkCallback() 이 매 프레임마다
   * SCB_InvalidateDCache_by_Addr() 를 호출하므로 정확히 이 경로에
   * 걸린다. 리셋 버튼으로는 캐시가 정리되어 증상이 안 나타나고,
   * 전원 인가 시에만 재현되는 이유이기도 하다.
   *
   * 참고: ST 커뮤니티 — STM32H743 Using Ethernet Without Cache Issue
   */
  SCB_InvalidateDCache();

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /*
   * D2 도메인 SRAM 클럭 활성화 — 이더넷보다 먼저 해야 한다.
   *
   * 이더넷 DMA 디스크립터(0x30000000), RX 버퍼 풀(0x30000100),
   * lwIP 힙(0x30005000)이 전부 D2 SRAM 에 있는데, 이 클럭은
   * 리셋 후 기본으로 꺼져 있다.
   *
   * CMSIS 의 SystemInit() 에도 활성화 코드가 있지만
   * #if defined(DATA_IN_D2_SRAM) 으로 감싸여 있고 이 프로젝트는
   * 그 매크로를 정의하지 않으므로 실행되지 않는다.
   *
   * 클럭이 꺼진 상태로도 잔류 전하 때문에 한동안 동작하는 것처럼
   * 보이다가, 버퍼된 쓰기가 실패하면서 IMPRECISERR(CFSR bit10)
   * HardFault 로 터진다. 부정확 폴트라 보고되는 PC 가 실제 원인과
   * 무관해 추적이 매우 어렵다.
   */
  __HAL_RCC_D2SRAM1_CLK_ENABLE();
  __HAL_RCC_D2SRAM2_CLK_ENABLE();
#if defined(RCC_AHB2ENR_D2SRAM3EN)
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
#endif

  EthernetMPU_Config();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();

  /* USER CODE BEGIN PHY_SETTLE */
  /*
   * PHY 안정화 지연 — 원인 격리를 위해 현재 비활성.
   * 필요 시 아래 주석을 풀어 사용한다.
   */
  /*
   * LAN8742 는 전원 인가 후 MDIO 응답까지 시간이 필요하다.
   * 데이터시트 기준 최소 약 26 ms 이나, ST 커뮤니티 다수 사례에서
   * 500 ms~2 s 가 필요했다. 리셋 버튼은 VDD 가 이미 안정된 상태라
   * 문제가 없지만, 전원 인가 경로에서는 이 지연이 없으면
   * PHY 열거(LAN8742_Init) 가 실패한다.
   */
  HAL_Delay(500U);
  /* USER CODE END PHY_SETTLE */

  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

#if (FRONT_INSTRUMENT_ENABLE)
  /* DWT 사이클 카운터 — RTOS 전환 전후 비교용 baseline 계측 */
  Perf_Init();

  /* 진단(DTC) — 'F' 존 */
  Diag_Init(front_dtc_defs, DTC_IDX_COUNT, 'F', Front_CaptureFreeze);
  dtc_brake_last_change_ms = HAL_GetTick();
#endif

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

#if (FRONT_INSTRUMENT_ENABLE)
      /* --- 계측: 메인 루프 1바퀴 주기 --- */
      {
          static uint32_t loop_prev_cyc = 0U;
          uint32_t cyc_now = PERF_NOW();

          if (loop_prev_cyc != 0U)
          {
              Perf_Update(&perf_loop_period, cyc_now - loop_prev_cyc);
          }
          loop_prev_cyc = cyc_now;
      }
#endif

      CAN_UpdateDiagnostics();
      CAN_BusOffRecover(now);


      if ((now - can250_last_tx_tick) >= CAN250_TX_PERIOD_MS)
      {
          can250_last_tx_tick = now;
          CAN250_SendPing();
      }

      FrontCommand_SendNext(now);

      FrontZoneNetwork_Process(now);

      /* Standalone(NO_SYS=1) LwIP packet, timeout, link processing */
#if (FRONT_INSTRUMENT_ENABLE)
      {
          uint32_t t0 = PERF_NOW();
          MX_LWIP_Process();
          Perf_Update(&perf_lwip_process, PERF_ELAPSED(t0));
      }
#else
      MX_LWIP_Process();
#endif

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

          /*
           * 계측: CAN 수신 -> UDP 송신 종단 지연
           * ft_tx_count 가 늘어난 루프에서만 유효한 측정이 된다.
           * (Process 는 내부 100 ms 주기라 매번 보내지 않음)
           */
#if (FRONT_INSTRUMENT_ENABLE)
          {
              uint32_t tx_before;

              /* 명령이 바뀐 순간에 시각을 찍는다. */
              if ((uint16_t)front_headlamp_command != refl_last_cmd)
              {
                  if (refl_last_cmd != 0xFFFFU)
                  {
                      /*
                       * 피드백이 이미 목표값이면 루프가 닫히는 순간을
                       * 관측할 수 없다(직전 지령이 아직 반영 중인 경우).
                       * 0 us 로 잘못 집계되므로 표본에서 뺀다.
                       */
                      if (f407_headlamp_output != front_headlamp_command)
                      {
                          refl_target  = front_headlamp_command;
                          refl_cycle   = PERF_NOW();
                          refl_pending = 1U;
                      }
                      else
                      {
                          refl_pending = 0U;
                      }
                  }
                  refl_last_cmd = (uint16_t)front_headlamp_command;
              }

              tx_before = ft_tx_count;
              FrontTelemetry_Process(&v);

              /*
               * 실제로 패킷이 나갔고, 그 패킷이 F407 이 되돌려준 값을
               * 담고 있을 때만 루프가 닫힌 것으로 본다.
               */
              if ((refl_pending != 0U) &&
                  (ft_tx_count != tx_before) &&
                  (v.headlamp == refl_target))
              {
                  Perf_Update(&perf_reflect_latency, PERF_ELAPSED(refl_cycle));
                  refl_pending = 0U;
              }
          }
#else
          FrontTelemetry_Process(&v);
#endif
      }

#if (FRONT_INSTRUMENT_ENABLE)
      /* 성능 통계를 Jetson 으로 1 Hz 송신 (RTOS 전후 비교용) */
      FrontTelemetry_SendPerf();

      /* 고장 진단 — 감지/디바운스 후 상태 변화 시 즉시 + 1 Hz 하트비트 */
      Front_ReportDiagnostics(now);
      FrontTelemetry_SendDtc(now);
#endif

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
    uint32_t perf_isr_t0;

    if (((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) ||
        (hfdcan->Instance != FDCAN2))
    {
        return;
    }

#if (FRONT_INSTRUMENT_ENABLE)
    /* --- 계측: ISR 소요시간 --- */
    perf_isr_t0 = PERF_NOW();
#else
    (void)perf_isr_t0;
#endif

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
            break;   /* return 대신 break — 아래 계측 갱신을 타도록 */
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

#if (FRONT_INSTRUMENT_ENABLE)
    /* --- 계측: ISR 소요시간 + 최신 CAN 데이터 도착 시각 --- */
    Perf_Update(&perf_can_isr, PERF_ELAPSED(perf_isr_t0));
    perf_can_stamp = PERF_NOW();
    perf_can_stamp_valid = 1U;
#endif
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
