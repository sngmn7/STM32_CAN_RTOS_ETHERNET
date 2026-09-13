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
#include "perf.h"
#include "diagnostic.h"
#include "eth_phy_recover.h"

/* 계측/진단 마스터 스위치 (Front 와 동일) */
#define REAR_INSTRUMENT_ENABLE   1
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
/* CAN 수신 두절 판정용 — 버스오프만으로는 상대 노드 소실을 못 잡는다. */
volatile uint32_t can500_last_rx_tick = 0U;
volatile uint32_t can250_last_rx_tick = 0U;
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

/* ---------------------------------------------------------------- */
/* 지연 계측 - Front 와 동일 방법론                                   */
/*                                                                    */
/*   perf_rear_cmd_latency     : 지령 변화 -> 하행 CAN 송신           */
/*   perf_rear_reflect_latency : 지령 변화 -> SR 텔레메트리 송신      */
/*                                                                    */
/* Rear 는 RearBody 로부터 출력 상태 피드백이 없어 텔레메트리가       */
/* 지령 자체를 싣는다. 따라서 두 경로를 각각 잰다.                    */
/* ---------------------------------------------------------------- */
volatile perf_stat_t perf_rear_cmd_latency     = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };
volatile perf_stat_t perf_rear_reflect_latency = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };

/* 지령 4종(좌/우/브레이크/윈도우)을 한 워드로 묶어 변화를 판정한다. */
static uint32_t rear_cmd_word_prev  = 0xFFFFFFFFU;  /* CAN 송신 기준 */
static uint32_t rear_cmd_word_tele  = 0xFFFFFFFFU;  /* 텔레메트리 기준 */
static uint32_t rear_cmd_cycle      = 0U;
static uint32_t rear_tele_cycle     = 0U;
static uint8_t  rear_cmd_pending    = 0U;
static uint8_t  rear_tele_pending   = 0U;
static uint8_t  rear_cmd_primed     = 0U;
static uint8_t  rear_tele_primed    = 0U;

static uint32_t Rear_CommandWord(void)
{
    return ((uint32_t)(led_left_command  ? 1U : 0U)      ) |
           ((uint32_t)(led_right_command ? 1U : 0U) <<  8) |
           ((uint32_t)(led_brake_command ? 1U : 0U) << 16) |
           ((uint32_t)(window_command    ? 1U : 0U) << 24);
}
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
static void CAN_BusOffRecover(uint32_t now);
static uint8_t CAN_LinkDead(uint32_t now, uint8_t busoff,
                            uint32_t rx_count, uint32_t last_rx_tick);

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

/* ------------------------------------------------------------------ */
/* 진단 (DTC) — Rear 존                                                */
/* confirm_ms 는 "정상 주기 x 3배 이상" 원칙 (COMM_ICD.md §4)          */
/* ------------------------------------------------------------------ */

enum {
    DTC_IDX_ZONE_LINK = 0,   /* Front <-> Rear 링크 상실   */
    DTC_IDX_HOST_LINK,       /* Jetson 명령 링크 상실      */
    DTC_IDX_CAN500_BUSOFF,   /* FDCAN1 버스오프            */
    DTC_IDX_CAN250_BUSOFF,   /* FDCAN2 버스오프            */
    DTC_IDX_ULTRA_LOST,      /* 초음파 0x312 무응답        */
    DTC_IDX_IMU_LOST,        /* IMU 0x301 무응답           */
    DTC_IDX_SWITCH_LOST,     /* 스위치 0x311 무응답        */
    DTC_IDX_COUNT
};

static const diag_def_t rear_dtc_defs[DTC_IDX_COUNT] = {
    /* code                        confirm  heal  warning */
    { DTC_U0100_ZONE_LINK_LOST,      1000U, 2000U, 1U },  /* 램프 제어 불능 */
    { DTC_U0300_HOST_LINK_LOST,      1500U, 3000U, 0U },
    { DTC_U0001_CAN500_BUSOFF,        500U, 3000U, 1U },
    { DTC_U0002_CAN250_BUSOFF,        500U, 3000U, 1U },  /* 램프 버스 */
    { DTC_P0500_ULTRASONIC_LOST,     2000U, 3000U, 1U },  /* 후방 감지 */
    { DTC_P0501_IMU_LOST,            5000U, 5000U, 0U },  /* 비주기 전송 */
    { DTC_P0502_SWITCH_LOST,         2000U, 3000U, 0U },
};

/*
 * 센서 무응답 판정 기준.
 *
 * 초음파 60 ms / 스위치 100 ms 는 주기적이라 1 s 면 충분하지만,
 * IMU(0x301)는 BNO085 리포트가 올 때만 전송되어 간격이 불규칙하다.
 * 1 s 로 두면 정상 동작 중에도 PENDING 이 떴다 사라지길 반복한다.
 * 실측 후 3 s 로 완화한다.
 */
#define REAR_SENSOR_LOST_MS   3000U
#define WINDOW_CMD_TX_PERIOD_MS   100U
/* 지령 변화 시 즉시 전송하되 이 간격보다 촘촘히는 보내지 않는다. */
#define REAR_CMD_MIN_GAP_MS        10U
/* 0 = 순수 주기 전송(개선 전, 비교 측정용) / 1 = 이벤트 트리거 */
#define REAR_EVENT_TRIGGER_ENABLE   1
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
 * @brief 프리즈 프레임 캡처 — 최초 DTC 확정 시점의 신호 스냅샷
 *
 * 슬롯 의미(Rear 존)는 COMM_ICD.md 에 정의.
 */
static void Rear_CaptureFreeze(uint16_t *sig)
{
    sig[0] = f446_ultrasonic_mm;
    sig[1] = (uint16_t)((uint16_t)f446_imu_data[0] |
                        ((uint16_t)f446_imu_data[1] << 8));
    sig[2] = (uint16_t)((uint16_t)f446_imu_data[2] |
                        ((uint16_t)f446_imu_data[3] << 8));
    sig[3] = rear_front_brake_adc;
    sig[4] = (uint16_t)((f446_switch_pressed ? 1U : 0U) |
                        (led_left_command  ? 2U : 0U) |
                        (led_right_command ? 4U : 0U) |
                        (led_brake_command ? 8U : 0U) |
                        (window_command    ? 16U : 0U));
    sig[5] = (uint16_t)((rear_front_link_alive     ? 1U : 0U) |
                        (rear_jetson_command_alive ? 2U : 0U) |
                        (can500_protocol_status.BusOff ? 4U : 0U) |
                        (can250_protocol_status.BusOff ? 8U : 0U));
    sig[6] = (uint16_t)(can500_rx_ok_count & 0xFFFFU);
    sig[7] = (uint16_t)(can250_rx_ok_count & 0xFFFFU);
}

/**
 * @brief 센서 무응답 판정
 *
 * last_tick 은 CAN ISR 이 갱신하고 now 는 메인 루프 시작에 캡처된다.
 * ISR 이 now 이후에 들어오면 last_tick > now 가 되어 부호 없는 뺄셈이
 * 언더플로우하고, 약 43억이 되어 오검출이 발생한다.
 * (증상: 정상 수신 중에도 PENDING 이 떴다 사라지길 반복)
 *
 * 부호 있는 비교로 "미래 시각 = 방금 수신" 을 걸러낸다.
 */
static uint8_t Rear_SensorLost(uint32_t last_tick, uint32_t now)
{
    if ((int32_t)(now - last_tick) < 0)
    {
        return 0U;   /* ISR 이 방금 갱신 — 정상 */
    }

    return ((uint32_t)(now - last_tick) > REAR_SENSOR_LOST_MS) ? 1U : 0U;
}

/**
 * @brief 원시 고장 여부를 진단 모듈에 보고 (관측만, 조치는 기존 로직)
 */
static void Rear_ReportDiagnostics(uint32_t now)
{
    Diag_Report(DTC_IDX_ZONE_LINK,
                (rear_front_link_alive == 0U) ? 1U : 0U, now);
    Diag_Report(DTC_IDX_HOST_LINK,
                (rear_jetson_command_alive == 0U) ? 1U : 0U, now);
    Diag_Report(DTC_IDX_CAN500_BUSOFF,
                CAN_LinkDead(now,
                             (uint8_t)(can500_protocol_status.BusOff != 0U),
                             can500_rx_count, can500_last_rx_tick), now);
    Diag_Report(DTC_IDX_CAN250_BUSOFF,
                CAN_LinkDead(now,
                             (uint8_t)(can250_protocol_status.BusOff != 0U),
                             can250_rx_count, can250_last_rx_tick), now);
    Diag_Report(DTC_IDX_ULTRA_LOST,
                Rear_SensorLost(f446_ultrasonic_last_tick, now), now);
    Diag_Report(DTC_IDX_IMU_LOST,
                Rear_SensorLost(f446_imu_last_tick, now), now);
    Diag_Report(DTC_IDX_SWITCH_LOST,
                Rear_SensorLost(f446_switch_last_tick, now), now);

    Diag_Process(now);
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

    {
        uint32_t w = Rear_CommandWord();
        uint32_t age = (uint32_t)(now - led_cmd_last_tx_tick);
        uint8_t  due;

        if (w != rear_cmd_word_prev)
        {
            if (rear_cmd_primed != 0U)
            {
                if (rear_cmd_pending == 0U)
                {
                    rear_cmd_cycle = PERF_NOW();
                    rear_cmd_pending = 1U;
                }
            }
            rear_cmd_primed = 1U;
            /* 변화 시 즉시 전송하되 최소 간격은 지킨다. */
#if (REAR_EVENT_TRIGGER_ENABLE)
            due = (age >= REAR_CMD_MIN_GAP_MS) ? 1U : 0U;
#else
            due = (age >= LED_CMD_TX_PERIOD_MS) ? 1U : 0U;
#endif
        }
        else
        {
            due = (age >= LED_CMD_TX_PERIOD_MS) ? 1U : 0U;
        }

        if (due != 0U)
        {
            uint32_t changed = w ^ rear_cmd_word_prev;

            led_cmd_last_tx_tick = now;
            rear_cmd_word_prev = w;
            LED_CommandSend();

            /* 윈도우는 값이 실제로 바뀐 경우에만 여기서 앞당겨 보낸다.
               그 외에는 Window_CommandUpdate() 의 주기 하트비트에 맡겨
               두 경로에서 중복 송신되지 않게 한다. */
            if ((changed & 0xFF000000U) != 0U)
            {
                window_cmd_last_tx_tick = now;
                Window_CommandSend();
            }

            if (rear_cmd_pending != 0U)
            {
                Perf_Update(&perf_rear_cmd_latency, PERF_ELAPSED(rear_cmd_cycle));
                rear_cmd_pending = 0U;
            }
        }
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

    /* 윈도우 지령은 LED_CommandUpdate() 에서 지령 워드와 함께 나간다.
       여기서는 값이 안 바뀌어도 살아있음을 알리는 하트비트만 담당한다. */
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
/**
 * @brief 버스오프 자동 복구
 *
 * FDCAN 은 버스오프에 빠지면 하드웨어가 CCCR.INIT 를 세우고 그대로 멈춘다.
 * 소프트웨어가 INIT 를 내려줘야 표준 복구 시퀀스(129 x 11 recessive bit)가
 * 시작된다. 내려주지 않으면 상대 노드가 살아 돌아와도 링크는 영구히 죽는다.
 *
 * 실제로 RearBody 가 리셋 고착에서 복귀했는데도 500k 링크가 살아나지
 * 않는 것을 계측으로 확인하고 추가했다. 진단(U0001)은 정상 동작했으나
 * 복구 경로가 없었다.
 *
 * 재시도 간격을 두는 이유는, 배선이 끊긴 채로 두면 복구 -> 즉시 버스오프를
 * 반복하며 버스를 흔들기 때문이다.
 */
#define CAN_BUSOFF_RECOVER_MS   1000U

volatile uint32_t can500_busoff_recover_count = 0U;
volatile uint32_t can250_busoff_recover_count = 0U;

/*
 * 링크 사망 판정 — 버스오프만 보면 안 된다.
 *
 * CAN 규격상 '에러 패시브 송신기가 ACK 오류를 검출했고 그동안 dominant 비트를
 * 보지 못했다면 TEC 를 올리지 않는다'. 그래서 상대 노드가 그냥 사라지면
 * TEC 가 128(에러 패시브 문턱)에 멈추고 버스오프(TEC>255)에 영원히 도달하지
 * 않는다. 실측으로 확인: TxErrorCnt=128, ErrorPassive=1, BusOff=0 인 채로
 * 링크가 완전히 죽어 있었고 DTC 는 정상으로 보고했다.
 *
 * 그래서 '수신 두절'을 함께 본다. 이더넷 링크(U0100/U0300)가 이미 쓰는
 * 판정 방식과 같다.
 */
#define CAN_RX_TIMEOUT_MS       2000U   /* 핑 주기 500 ms 의 4배 */
#define CAN_STARTUP_GRACE_MS    3000U   /* 기동 직후 유예 */

static uint8_t CAN_LinkDead(uint32_t now, uint8_t busoff,
                            uint32_t rx_count, uint32_t last_rx_tick)
{
    if (busoff != 0U)
    {
        return 1U;
    }
    if (rx_count == 0U)
    {
        /* 부팅 후 한 번도 못 받았다 - 유예시간이 지나면 고장 */
        return (now > CAN_STARTUP_GRACE_MS) ? 1U : 0U;
    }
    if ((int32_t)(now - last_rx_tick) < 0)
    {
        return 0U;   /* ISR 이 방금 갱신 */
    }
    return ((uint32_t)(now - last_rx_tick) > CAN_RX_TIMEOUT_MS) ? 1U : 0U;
}
static void CAN_BusOffRecover(uint32_t now)
{
    static uint32_t can500_last_try = 0U;
    static uint32_t can250_last_try = 0U;

    if (can500_protocol_status.BusOff != 0U)
    {
        if ((uint32_t)(now - can500_last_try) >= CAN_BUSOFF_RECOVER_MS)
        {
            can500_last_try = now;
            CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
            can500_busoff_recover_count++;
        }
    }

    if (can250_protocol_status.BusOff != 0U)
    {
        if ((uint32_t)(now - can250_last_try) >= CAN_BUSOFF_RECOVER_MS)
        {
            can250_last_try = now;
            CLEAR_BIT(hfdcan2.Instance->CCCR, FDCAN_CCCR_INIT);
            can250_busoff_recover_count++;
        }
    }
}
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
  MX_FDCAN1_Init();
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

#if (REAR_INSTRUMENT_ENABLE)
  Perf_Init();
  Diag_Init(rear_dtc_defs, DTC_IDX_COUNT, 'R', Rear_CaptureFreeze);
#endif

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
      CAN_BusOffRecover(now);

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

#if (REAR_INSTRUMENT_ENABLE)
      {
          static uint32_t loop_prev_cyc = 0U;
          uint32_t cyc_now = PERF_NOW();

          if (loop_prev_cyc != 0U)
          {
              Perf_Update(&perf_loop_period, cyc_now - loop_prev_cyc);
          }
          loop_prev_cyc = cyc_now;
      }
      /* Standalone(NO_SYS=1) LwIP packet, timeout, link processing */
      {
          uint32_t t0 = PERF_NOW();
          MX_LWIP_Process();
          Perf_Update(&perf_lwip_process, PERF_ELAPSED(t0));
      }
#else
      /* Standalone(NO_SYS=1) LwIP packet, timeout, link processing */
      MX_LWIP_Process();
#endif

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

#if (REAR_INSTRUMENT_ENABLE)
          {
              uint32_t w = Rear_CommandWord();
              uint32_t tx_before;

              if (w != rear_cmd_word_tele)
              {
                  if (rear_tele_primed != 0U)
                  {
                      if (rear_tele_pending == 0U)
                      {
                          rear_tele_cycle = PERF_NOW();
                          rear_tele_pending = 1U;
                      }
                  }
                  rear_tele_primed = 1U;
                  rear_cmd_word_tele = w;
              }

              tx_before = rt_tx_count;
              RearTelemetry_Process(&v);

              /* 실제로 패킷이 나갔을 때만 루프가 닫힌 것으로 본다. */
              if ((rear_tele_pending != 0U) && (rt_tx_count != tx_before))
              {
                  Perf_Update(&perf_rear_reflect_latency,
                              PERF_ELAPSED(rear_tele_cycle));
                  rear_tele_pending = 0U;
              }
          }
#else
          RearTelemetry_Process(&v);
#endif
      }

#if (REAR_INSTRUMENT_ENABLE)
      RearTelemetry_SendPerf();
      Rear_ReportDiagnostics(now);
      RearTelemetry_SendDtc(now);
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
            can500_last_rx_tick = HAL_GetTick();

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
            can250_last_rx_tick = HAL_GetTick();

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
