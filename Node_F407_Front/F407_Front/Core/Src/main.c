/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Front lower node integrated application
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t raw_active;
  uint8_t stable_active;
  uint32_t last_change_tick;
} DebouncedInput_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VALUE_COUNT                 3U
#define ADC_INDEX_STEERING              0U
#define ADC_INDEX_BRAKE                 1U
#define ADC_INDEX_ACCEL                 2U
#define ADC_FULL_SCALE                  4095U

#define CONTROL_UPDATE_PERIOD_MS        10U
#define SWITCH_DEBOUNCE_MS              20U

#define ACTUATOR_MAX_STEP               4095
#define ACTUATOR_DEADBAND_STEP          2
#define ACTUATOR_STEP_SEQUENCE_COUNT    8

#define CAN_ID_H723_PING                         0x201U
#define CAN_ID_F407_ECHO                         0x281U

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

/* H723 -> F407: one command per Standard CAN ID */
#define CAN_ID_H723_STEERING_OVERRIDE            0x310U
#define CAN_ID_H723_ACTUATOR_ENABLE              0x311U
#define CAN_ID_H723_ACTUATOR_TARGET              0x312U
#define CAN_ID_H723_HEADLAMP_OVERRIDE            0x313U
#define CAN_ID_H723_HEADLAMP_COMMAND             0x314U
#define CAN_ID_H723_TURN_OVERRIDE                0x315U
#define CAN_ID_H723_TURN_MODE                    0x316U

#define CAN_PERIOD_ADC_MS                         20U
#define CAN_PERIOD_ACTUATOR_POSITION_MS           20U
#define CAN_PERIOD_DIGITAL_STATUS_MS              50U
#define CAN_PERIOD_CAN_DIAGNOSTIC_SIGNAL_MS      100U
#define CAN_PERIOD_ENVIRONMENT_MS               1000U
#define CAN_DIAGNOSTIC_PERIOD_MS                 100U
#define CAN_COMMAND_TIMEOUT_MS                   500U

#define DHT11_READ_PERIOD_MS            2000U
#define DHT11_START_LOW_MS              18U
#define DHT11_TIMEOUT_US                120U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LIMIT_U16(value, maximum) \
  ((uint16_t)(((value) > (maximum)) ? (maximum) : (value)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ADC1 DMA 순서: PA1 Steering, PC1 Brake, PC2 Accel */
volatile uint16_t adc_dma_values[ADC_VALUE_COUNT] = {0U, 0U, 0U};
volatile uint16_t steering_adc_raw = 0U;
volatile uint16_t steering_adc_filtered = 0U;
volatile uint16_t brake_adc_raw = 0U;
volatile uint16_t accel_adc_raw = 0U;
volatile uint32_t adc_dma_error_count = 0U;
static int32_t steering_filter_accumulator = 0;
static uint8_t steering_filter_initialized = 0U;

/*
 * PE9 active-low push button cycles the turn lamps:
 * 0: both OFF -> 1: PE7 left ON -> 2: PE8 right ON -> 0: both OFF.
 */
volatile uint8_t turn_switch_pressed = 0U;
volatile uint8_t turn_mode = 0U;          /* applied mode */
volatile uint8_t turn_mode_local = 0U;
volatile uint8_t can_turn_override_enable = 0U;
volatile uint8_t can_turn_mode_command = 0U;
volatile uint8_t turn_override_active = 0U;
volatile uint8_t turn_left_output_state = 0U;
volatile uint8_t turn_right_output_state = 0U;
volatile uint8_t headlamp_output_state = 0U;

static uint8_t turn_switch_previous_pressed = 0U;
static DebouncedInput_t turn_switch_input = {
  TURN_SWITCH_GPIO_Port, TURN_SWITCH_Pin, 0U, 0U, 0U
};

/* 28BYJ/4-input half-step actuator control */
static const uint8_t actuator_step_sequence[ACTUATOR_STEP_SEQUENCE_COUNT][4] = {
  {1U, 0U, 0U, 0U},
  {1U, 1U, 0U, 0U},
  {0U, 1U, 0U, 0U},
  {0U, 1U, 1U, 0U},
  {0U, 0U, 1U, 0U},
  {0U, 0U, 1U, 1U},
  {0U, 0U, 0U, 1U},
  {1U, 0U, 0U, 1U}
};

volatile int32_t actuator_absolute_position = 0;
volatile int32_t actuator_target_position = 0;
volatile int32_t actuator_local_target_position = 0;
volatile uint8_t actuator_step_index = 0U;
volatile uint8_t actuator_local_enable = 1U;
volatile uint8_t actuator_output_enable = 0U;
volatile uint32_t actuator_step_count = 0U;
volatile uint32_t actuator_limit_count = 0U;

/* DHT11 diagnostics and values are stored in 0.1 units. */
volatile int16_t dht11_temperature_x10 = 0;
volatile uint16_t dht11_humidity_x10 = 0U;
volatile uint8_t dht11_valid = 0U;
volatile uint8_t dht11_last_checksum = 0U;
volatile uint32_t dht11_ok_count = 0U;
volatile uint32_t dht11_error_count = 0U;

/* CAN diagnostics */
volatile uint8_t can_started = 0U;
volatile uint32_t can_start_error_step = 0U;
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_ping_rx_count = 0U;
volatile uint32_t can_echo_tx_count = 0U;
volatile uint32_t can_command_rx_count = 0U;
volatile uint32_t can_unknown_rx_count = 0U;
volatile uint32_t can_tx_busy_count = 0U;
volatile uint32_t can_tx_error_count = 0U;
volatile uint32_t can_error_callback_count = 0U;
volatile uint32_t can_busoff_count = 0U;
volatile uint32_t can_last_error = 0U;
volatile uint32_t can_last_rx_id = 0U;
volatile uint8_t can_last_rx_dlc = 0U;
volatile uint8_t can_last_rx_data[8] = {0U};
volatile uint32_t can_tx_mailbox_free = 0U;
volatile uint32_t can_esr_register = 0U;
volatile uint8_t can_tx_error_counter = 0U;
volatile uint8_t can_rx_error_counter = 0U;
volatile uint8_t can_last_error_code = 0U;
volatile uint8_t can_busoff_state = 0U;

/* Per-signal F407 -> H723 transmit counters */
volatile uint32_t can_steering_tx_count = 0U;
volatile uint32_t can_brake_tx_count = 0U;
volatile uint32_t can_accel_tx_count = 0U;
volatile uint32_t can_turn_left_tx_count = 0U;
volatile uint32_t can_turn_right_tx_count = 0U;
volatile uint32_t can_turn_switch_tx_count = 0U;
volatile uint32_t can_headlamp_output_tx_count = 0U;
volatile uint32_t can_temperature_tx_count = 0U;
volatile uint32_t can_humidity_tx_count = 0U;
volatile uint32_t can_dht11_valid_tx_count = 0U;
volatile uint32_t can_actuator_position_tx_count = 0U;
volatile uint32_t can_actuator_enable_tx_count = 0U;
volatile uint32_t can_steering_override_tx_count = 0U;
volatile uint32_t can_headlamp_override_tx_count = 0U;
volatile uint32_t can_busoff_signal_tx_count = 0U;

/* Per-command H723 -> F407 receive counters */
volatile uint32_t can_steering_override_rx_count = 0U;
volatile uint32_t can_actuator_enable_rx_count = 0U;
volatile uint32_t can_actuator_target_rx_count = 0U;
volatile uint32_t can_headlamp_override_rx_count = 0U;
volatile uint32_t can_headlamp_command_rx_count = 0U;
volatile uint32_t can_turn_override_rx_count = 0U;
volatile uint32_t can_turn_mode_rx_count = 0U;

/* H723 ping echo is queued in the RX callback and transmitted in main context. */
volatile uint8_t can_echo_pending = 0U;
static uint8_t can_echo_data[8] = {0U};

/* Split command state and individual freshness timestamps */
volatile uint8_t can_steering_override_enable = 0U;
volatile uint8_t can_actuator_enable_command = 0U;
volatile uint8_t can_headlamp_override_enable = 0U;
volatile uint8_t can_headlamp_command = 0U;
volatile uint16_t can_actuator_target_command = 0U;
volatile uint32_t can_steering_override_last_tick = 0U;
volatile uint32_t can_actuator_enable_last_tick = 0U;
volatile uint32_t can_actuator_target_last_tick = 0U;
volatile uint32_t can_headlamp_override_last_tick = 0U;
volatile uint32_t can_headlamp_command_last_tick = 0U;
volatile uint32_t can_turn_override_last_tick = 0U;
volatile uint32_t can_turn_mode_last_tick = 0U;

/* Per-signal scheduler state. At most one status frame is queued per main-loop pass. */
static uint8_t can_signal_scheduler_index = 0U;
static uint32_t can_steering_last_tx_tick = 0U;
static uint32_t can_brake_last_tx_tick = 0U;
static uint32_t can_accel_last_tx_tick = 0U;
static uint32_t can_turn_left_last_tx_tick = 0U;
static uint32_t can_turn_right_last_tx_tick = 0U;
static uint32_t can_turn_switch_last_tx_tick = 0U;
static uint32_t can_headlamp_output_last_tx_tick = 0U;
static uint32_t can_temperature_last_tx_tick = 0U;
static uint32_t can_humidity_last_tx_tick = 0U;
static uint32_t can_dht11_valid_last_tx_tick = 0U;
static uint32_t can_actuator_position_last_tx_tick = 0U;
static uint32_t can_actuator_enable_last_tx_tick = 0U;
static uint32_t can_steering_override_last_tx_tick = 0U;
static uint32_t can_headlamp_override_last_tx_tick = 0U;
static uint32_t can_busoff_signal_last_tx_tick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void DWT_DelayInit(void);
static void DWT_DelayUs(uint32_t delay_us);
static uint32_t DWT_ElapsedUs(uint32_t start_cycle);

static void Switch_Init(DebouncedInput_t *input, uint32_t now);
static void Switch_Update(DebouncedInput_t *input, uint32_t now);
static void Application_UpdateInputsAndOutputs(uint32_t now);

static void Actuator_WriteStep(uint8_t index);
static void Actuator_SetEnabled(uint8_t enable);
static void Actuator_AllOutputsLow(void);

static void DHT11_SetPinOutput(void);
static void DHT11_SetPinInput(void);
static uint8_t DHT11_WaitWhile(GPIO_PinState state, uint32_t timeout_us);
static uint8_t DHT11_ReadSensor(void);

static void CAN1_ApplicationStart(void);
static HAL_StatusTypeDef CAN1_SendStandard(uint32_t identifier,
                                           const uint8_t data[8],
                                           uint8_t dlc);
static void CAN1_SendPendingEcho(void);
static HAL_StatusTypeDef CAN1_SendU8Signal(uint32_t identifier, uint8_t value);
static HAL_StatusTypeDef CAN1_SendU16Signal(uint32_t identifier, uint16_t value);
static HAL_StatusTypeDef CAN1_SendI16Signal(uint32_t identifier, int16_t value);
static void CAN1_SignalSchedulerInit(uint32_t now);
static void CAN1_SendNextSignal(uint32_t now);
static void CAN1_UpdateDiagnostics(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void DWT_DelayInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DWT_DelayUs(uint32_t delay_us)
{
  uint32_t start_cycle = DWT->CYCCNT;
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  uint32_t wait_cycles = delay_us * cycles_per_us;

  while ((uint32_t)(DWT->CYCCNT - start_cycle) < wait_cycles)
  {
  }
}

static uint32_t DWT_ElapsedUs(uint32_t start_cycle)
{
  uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U)
  {
    return 0U;
  }

  return (uint32_t)(DWT->CYCCNT - start_cycle) / cycles_per_us;
}

static void Switch_Init(DebouncedInput_t *input, uint32_t now)
{
  uint8_t active;

  active = (HAL_GPIO_ReadPin(input->port, input->pin) == GPIO_PIN_RESET) ? 1U : 0U;
  input->raw_active = active;
  input->stable_active = active;
  input->last_change_tick = now;
}

static void Switch_Update(DebouncedInput_t *input, uint32_t now)
{
  uint8_t active;

  active = (HAL_GPIO_ReadPin(input->port, input->pin) == GPIO_PIN_RESET) ? 1U : 0U;

  if (active != input->raw_active)
  {
    input->raw_active = active;
    input->last_change_tick = now;
  }

  if ((uint32_t)(now - input->last_change_tick) >= SWITCH_DEBOUNCE_MS)
  {
    input->stable_active = input->raw_active;
  }
}

static void Actuator_AllOutputsLow(void)
{
  HAL_GPIO_WritePin(ACT_IN1_GPIO_Port, ACT_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN2_GPIO_Port, ACT_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN3_GPIO_Port, ACT_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN4_GPIO_Port, ACT_IN4_Pin, GPIO_PIN_RESET);
}

static void Actuator_SetEnabled(uint8_t enable)
{
  actuator_output_enable = (enable != 0U) ? 1U : 0U;

  if (actuator_output_enable != 0U)
  {
    /* Existing F401 hardware used active-high ENA/ENB. */
    HAL_GPIO_WritePin(ACT_ENA_GPIO_Port, ACT_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ACT_ENB_GPIO_Port, ACT_ENB_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(ACT_ENA_GPIO_Port, ACT_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ACT_ENB_GPIO_Port, ACT_ENB_Pin, GPIO_PIN_RESET);
    Actuator_AllOutputsLow();
  }
}

static void Actuator_WriteStep(uint8_t index)
{
  uint8_t safe_index = (uint8_t)(index % ACTUATOR_STEP_SEQUENCE_COUNT);

  HAL_GPIO_WritePin(ACT_IN1_GPIO_Port,
                    ACT_IN1_Pin,
                    actuator_step_sequence[safe_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN2_GPIO_Port,
                    ACT_IN2_Pin,
                    actuator_step_sequence[safe_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN3_GPIO_Port,
                    ACT_IN3_Pin,
                    actuator_step_sequence[safe_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ACT_IN4_GPIO_Port,
                    ACT_IN4_Pin,
                    actuator_step_sequence[safe_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Application_UpdateInputsAndOutputs(uint32_t now)
{
  uint16_t steering_sample;
  int32_t target;
  uint8_t command_is_fresh;
  uint8_t requested_actuator_enable;
  uint8_t requested_headlamp_state;

  steering_adc_raw = adc_dma_values[ADC_INDEX_STEERING];
  brake_adc_raw = adc_dma_values[ADC_INDEX_BRAKE];
  accel_adc_raw = adc_dma_values[ADC_INDEX_ACCEL];

  steering_sample = LIMIT_U16(steering_adc_raw, ADC_FULL_SCALE);

  if (steering_filter_initialized == 0U)
  {
    steering_filter_accumulator = ((int32_t)steering_sample << 4);
    steering_filter_initialized = 1U;
  }
  else
  {
    /* First-order low-pass filter. */
    steering_filter_accumulator +=
      ((((int32_t)steering_sample << 4) - steering_filter_accumulator) / 8);
  }

  steering_adc_filtered = (uint16_t)(steering_filter_accumulator >> 4);
  target = ((int32_t)steering_adc_filtered * ACTUATOR_MAX_STEP) / (int32_t)ADC_FULL_SCALE;
  actuator_local_target_position = target;

  Switch_Update(&turn_switch_input, now);
  turn_switch_pressed = turn_switch_input.stable_active;

  /* Local button advances only once on each debounced press edge. */
  if ((turn_switch_pressed != 0U) &&
      (turn_switch_previous_pressed == 0U))
  {
    turn_mode_local++;
    if (turn_mode_local >= 3U)
    {
      turn_mode_local = 0U;
    }
  }
  turn_switch_previous_pressed = turn_switch_pressed;

  command_is_fresh =
    (((uint32_t)(now - can_turn_override_last_tick) <= CAN_COMMAND_TIMEOUT_MS) &&
     ((uint32_t)(now - can_turn_mode_last_tick) <= CAN_COMMAND_TIMEOUT_MS)) ? 1U : 0U;

  if (command_is_fresh == 0U)
  {
    can_turn_override_enable = 0U;
  }

  if ((command_is_fresh != 0U) && (can_turn_override_enable != 0U))
  {
    turn_mode = (can_turn_mode_command <= 2U) ? can_turn_mode_command : 0U;
    turn_override_active = 1U;
  }
  else
  {
    turn_mode = turn_mode_local;
    turn_override_active = 0U;
  }

  if (turn_mode == 1U)
  {
    turn_left_output_state = 1U;
    turn_right_output_state = 0U;
  }
  else if (turn_mode == 2U)
  {
    turn_left_output_state = 0U;
    turn_right_output_state = 1U;
  }
  else
  {
    turn_left_output_state = 0U;
    turn_right_output_state = 0U;
  }

  HAL_GPIO_WritePin(TURN_LEFT_LED_GPIO_Port,
                    TURN_LEFT_LED_Pin,
                    turn_left_output_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TURN_RIGHT_LED_GPIO_Port,
                    TURN_RIGHT_LED_Pin,
                    turn_right_output_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

  command_is_fresh =
    (((uint32_t)(now - can_steering_override_last_tick) <= CAN_COMMAND_TIMEOUT_MS) &&
     ((uint32_t)(now - can_actuator_enable_last_tick) <= CAN_COMMAND_TIMEOUT_MS) &&
     ((uint32_t)(now - can_actuator_target_last_tick) <= CAN_COMMAND_TIMEOUT_MS)) ? 1U : 0U;

  if (command_is_fresh == 0U)
  {
    can_steering_override_enable = 0U;
  }

  if ((command_is_fresh != 0U) && (can_steering_override_enable != 0U))
  {
    actuator_target_position = (int32_t)can_actuator_target_command;
    requested_actuator_enable = can_actuator_enable_command;
  }
  else
  {
    actuator_target_position = actuator_local_target_position;
    requested_actuator_enable = actuator_local_enable;
  }

  if (requested_actuator_enable != actuator_output_enable)
  {
    Actuator_SetEnabled(requested_actuator_enable);
  }

  command_is_fresh =
    (((uint32_t)(now - can_headlamp_override_last_tick) <= CAN_COMMAND_TIMEOUT_MS) &&
     ((uint32_t)(now - can_headlamp_command_last_tick) <= CAN_COMMAND_TIMEOUT_MS)) ? 1U : 0U;

  if (command_is_fresh == 0U)
  {
    can_headlamp_override_enable = 0U;
  }

  if ((command_is_fresh != 0U) && (can_headlamp_override_enable != 0U))
  {
    requested_headlamp_state = can_headlamp_command;
  }
  else
  {
    /* No local headlamp switch is assigned; PE10 defaults safely to OFF. */
    requested_headlamp_state = 0U;
  }

  headlamp_output_state = (requested_headlamp_state != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port,
                    HEADLAMP_OUT_Pin,
                    headlamp_output_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void DHT11_SetPinOutput(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  gpio_init.Pin = DHT11_DATA_Pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &gpio_init);
}

static void DHT11_SetPinInput(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  gpio_init.Pin = DHT11_DATA_Pin;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_DATA_GPIO_Port, &gpio_init);
}

static uint8_t DHT11_WaitWhile(GPIO_PinState state, uint32_t timeout_us)
{
  uint32_t start_cycle = DWT->CYCCNT;

  while (HAL_GPIO_ReadPin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin) == state)
  {
    if (DWT_ElapsedUs(start_cycle) >= timeout_us)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t DHT11_ReadSensor(void)
{
  uint8_t data[5] = {0U, 0U, 0U, 0U, 0U};
  uint8_t bit_index;
  uint8_t byte_index;
  uint32_t high_start_cycle;
  uint32_t high_time_us;
  uint8_t success = 0U;

  /* Keep DHT11 pulse measurements deterministic for about 20 ms. */
  HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);

  DHT11_SetPinOutput();
  HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_RESET);
  HAL_Delay(DHT11_START_LOW_MS);
  HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);
  DWT_DelayUs(30U);
  DHT11_SetPinInput();

  /* Sensor response: 80 us LOW, 80 us HIGH. */
  if (DHT11_WaitWhile(GPIO_PIN_SET, DHT11_TIMEOUT_US) == 0U)
  {
    goto dht11_exit;
  }
  if (DHT11_WaitWhile(GPIO_PIN_RESET, DHT11_TIMEOUT_US) == 0U)
  {
    goto dht11_exit;
  }
  if (DHT11_WaitWhile(GPIO_PIN_SET, DHT11_TIMEOUT_US) == 0U)
  {
    goto dht11_exit;
  }

  for (bit_index = 0U; bit_index < 40U; bit_index++)
  {
    if (DHT11_WaitWhile(GPIO_PIN_RESET, DHT11_TIMEOUT_US) == 0U)
    {
      goto dht11_exit;
    }

    high_start_cycle = DWT->CYCCNT;
    if (DHT11_WaitWhile(GPIO_PIN_SET, DHT11_TIMEOUT_US) == 0U)
    {
      goto dht11_exit;
    }
    high_time_us = DWT_ElapsedUs(high_start_cycle);

    byte_index = bit_index / 8U;
    data[byte_index] <<= 1U;
    if (high_time_us > 50U)
    {
      data[byte_index] |= 1U;
    }
  }

  dht11_last_checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (dht11_last_checksum != data[4])
  {
    goto dht11_exit;
  }

  dht11_humidity_x10 = (uint16_t)(((uint16_t)data[0] * 10U) + data[1]);
  dht11_temperature_x10 =
    (int16_t)(((int16_t)(data[2] & 0x7FU) * 10) + (int16_t)data[3]);

  if ((data[2] & 0x80U) != 0U)
  {
    dht11_temperature_x10 = (int16_t)(-dht11_temperature_x10);
  }

  success = 1U;

dht11_exit:
  DHT11_SetPinOutput();
  HAL_GPIO_WritePin(DHT11_DATA_GPIO_Port, DHT11_DATA_Pin, GPIO_PIN_SET);

  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

  if (success != 0U)
  {
    dht11_valid = 1U;
    dht11_ok_count++;
  }
  else
  {
    dht11_valid = 0U;
    dht11_error_count++;
  }

  return success;
}

static void CAN1_ApplicationStart(void)
{
  CAN_FilterTypeDef filter = {0};
  uint32_t notifications;

  /* 32-bit ID-mask zero accepts all CAN frames into FIFO0. */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    can_start_error_step = 1U;
    can_last_error = HAL_CAN_GetError(&hcan1);
    return;
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    can_start_error_step = 2U;
    can_last_error = HAL_CAN_GetError(&hcan1);
    return;
  }

  notifications = CAN_IT_RX_FIFO0_MSG_PENDING |
                  CAN_IT_RX_FIFO0_OVERRUN |
                  CAN_IT_ERROR_WARNING |
                  CAN_IT_ERROR_PASSIVE |
                  CAN_IT_BUSOFF |
                  CAN_IT_LAST_ERROR_CODE |
                  CAN_IT_ERROR;

  if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK)
  {
    can_start_error_step = 3U;
    can_last_error = HAL_CAN_GetError(&hcan1);
    return;
  }

  can_started = 1U;
  can_start_error_step = 0U;
}

static HAL_StatusTypeDef CAN1_SendStandard(uint32_t identifier,
                                           const uint8_t data[8],
                                           uint8_t dlc)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox = 0U;
  HAL_StatusTypeDef status;

  if ((can_started == 0U) || (dlc > 8U))
  {
    return HAL_ERROR;
  }

  can_tx_mailbox_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
  if (can_tx_mailbox_free == 0U)
  {
    can_tx_busy_count++;
    can_last_error = HAL_CAN_GetError(&hcan1);
    return HAL_BUSY;
  }

  header.StdId = identifier & 0x7FFU;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  status = HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
  if (status != HAL_OK)
  {
    can_tx_error_count++;
    can_last_error = HAL_CAN_GetError(&hcan1);
  }

  return status;
}

static void CAN1_SendPendingEcho(void)
{
  if (can_echo_pending == 0U)
  {
    return;
  }

  if (CAN1_SendStandard(CAN_ID_F407_ECHO, can_echo_data, 8U) == HAL_OK)
  {
    can_echo_pending = 0U;
    can_echo_tx_count++;
  }
}

static HAL_StatusTypeDef CAN1_SendU8Signal(uint32_t identifier, uint8_t value)
{
  uint8_t data[8] = {0U};
  data[0] = value;
  return CAN1_SendStandard(identifier, data, 1U);
}

static HAL_StatusTypeDef CAN1_SendU16Signal(uint32_t identifier, uint16_t value)
{
  uint8_t data[8] = {0U};
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8U) & 0xFFU);
  return CAN1_SendStandard(identifier, data, 2U);
}

static HAL_StatusTypeDef CAN1_SendI16Signal(uint32_t identifier, int16_t value)
{
  return CAN1_SendU16Signal(identifier, (uint16_t)value);
}

static void CAN1_SignalSchedulerInit(uint32_t now)
{
  can_steering_last_tx_tick = now - CAN_PERIOD_ADC_MS;
  can_brake_last_tx_tick = now - CAN_PERIOD_ADC_MS;
  can_accel_last_tx_tick = now - CAN_PERIOD_ADC_MS;
  can_turn_left_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_turn_right_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_turn_switch_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_headlamp_output_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_temperature_last_tx_tick = now - CAN_PERIOD_ENVIRONMENT_MS;
  can_humidity_last_tx_tick = now - CAN_PERIOD_ENVIRONMENT_MS;
  can_dht11_valid_last_tx_tick = now - CAN_PERIOD_ENVIRONMENT_MS;
  can_actuator_position_last_tx_tick = now - CAN_PERIOD_ACTUATOR_POSITION_MS;
  can_actuator_enable_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_steering_override_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_headlamp_override_last_tx_tick = now - CAN_PERIOD_DIGITAL_STATUS_MS;
  can_busoff_signal_last_tx_tick = now - CAN_PERIOD_CAN_DIAGNOSTIC_SIGNAL_MS;
  can_signal_scheduler_index = 0U;
}

static void CAN1_SendNextSignal(uint32_t now)
{
  uint8_t attempt;
  uint16_t actuator_position;
  HAL_StatusTypeDef status;

  for (attempt = 0U; attempt < 15U; attempt++)
  {
    status = HAL_ERROR;

    switch (can_signal_scheduler_index)
    {
      case 0U:
        if ((uint32_t)(now - can_steering_last_tx_tick) >= CAN_PERIOD_ADC_MS)
        {
          status = CAN1_SendU16Signal(CAN_ID_F407_STEERING_ADC, steering_adc_filtered);
          if (status == HAL_OK)
          {
            can_steering_last_tx_tick = now;
            can_steering_tx_count++;
          }
        }
        break;

      case 1U:
        if ((uint32_t)(now - can_brake_last_tx_tick) >= CAN_PERIOD_ADC_MS)
        {
          status = CAN1_SendU16Signal(CAN_ID_F407_BRAKE_ADC, brake_adc_raw);
          if (status == HAL_OK)
          {
            can_brake_last_tx_tick = now;
            can_brake_tx_count++;
          }
        }
        break;

      case 2U:
        if ((uint32_t)(now - can_accel_last_tx_tick) >= CAN_PERIOD_ADC_MS)
        {
          status = CAN1_SendU16Signal(CAN_ID_F407_ACCEL_ADC, accel_adc_raw);
          if (status == HAL_OK)
          {
            can_accel_last_tx_tick = now;
            can_accel_tx_count++;
          }
        }
        break;

      case 3U:
        if ((uint32_t)(now - can_turn_left_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_TURN_LEFT_OUTPUT, turn_left_output_state);
          if (status == HAL_OK)
          {
            can_turn_left_last_tx_tick = now;
            can_turn_left_tx_count++;
          }
        }
        break;

      case 4U:
        if ((uint32_t)(now - can_turn_right_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_TURN_RIGHT_OUTPUT, turn_right_output_state);
          if (status == HAL_OK)
          {
            can_turn_right_last_tx_tick = now;
            can_turn_right_tx_count++;
          }
        }
        break;

      case 5U:
        if ((uint32_t)(now - can_turn_switch_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_TURN_SWITCH_PRESSED, turn_switch_pressed);
          if (status == HAL_OK)
          {
            can_turn_switch_last_tx_tick = now;
            can_turn_switch_tx_count++;
          }
        }
        break;

      case 6U:
        if ((uint32_t)(now - can_headlamp_output_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_HEADLAMP_OUTPUT, headlamp_output_state);
          if (status == HAL_OK)
          {
            can_headlamp_output_last_tx_tick = now;
            can_headlamp_output_tx_count++;
          }
        }
        break;

      case 7U:
        if ((uint32_t)(now - can_temperature_last_tx_tick) >= CAN_PERIOD_ENVIRONMENT_MS)
        {
          status = CAN1_SendI16Signal(CAN_ID_F407_TEMPERATURE_X10, dht11_temperature_x10);
          if (status == HAL_OK)
          {
            can_temperature_last_tx_tick = now;
            can_temperature_tx_count++;
          }
        }
        break;

      case 8U:
        if ((uint32_t)(now - can_humidity_last_tx_tick) >= CAN_PERIOD_ENVIRONMENT_MS)
        {
          status = CAN1_SendU16Signal(CAN_ID_F407_HUMIDITY_X10, dht11_humidity_x10);
          if (status == HAL_OK)
          {
            can_humidity_last_tx_tick = now;
            can_humidity_tx_count++;
          }
        }
        break;

      case 9U:
        if ((uint32_t)(now - can_dht11_valid_last_tx_tick) >= CAN_PERIOD_ENVIRONMENT_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_DHT11_VALID, dht11_valid);
          if (status == HAL_OK)
          {
            can_dht11_valid_last_tx_tick = now;
            can_dht11_valid_tx_count++;
          }
        }
        break;

      case 10U:
        if ((uint32_t)(now - can_actuator_position_last_tx_tick) >= CAN_PERIOD_ACTUATOR_POSITION_MS)
        {
          if (actuator_absolute_position < 0)
          {
            actuator_position = 0U;
          }
          else if (actuator_absolute_position > ACTUATOR_MAX_STEP)
          {
            actuator_position = (uint16_t)ACTUATOR_MAX_STEP;
          }
          else
          {
            actuator_position = (uint16_t)actuator_absolute_position;
          }

          status = CAN1_SendU16Signal(CAN_ID_F407_ACTUATOR_POSITION, actuator_position);
          if (status == HAL_OK)
          {
            can_actuator_position_last_tx_tick = now;
            can_actuator_position_tx_count++;
          }
        }
        break;

      case 11U:
        if ((uint32_t)(now - can_actuator_enable_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_ACTUATOR_ENABLE, actuator_output_enable);
          if (status == HAL_OK)
          {
            can_actuator_enable_last_tx_tick = now;
            can_actuator_enable_tx_count++;
          }
        }
        break;

      case 12U:
        if ((uint32_t)(now - can_steering_override_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_STEERING_OVERRIDE_ACTIVE,
                                     can_steering_override_enable);
          if (status == HAL_OK)
          {
            can_steering_override_last_tx_tick = now;
            can_steering_override_tx_count++;
          }
        }
        break;

      case 13U:
        if ((uint32_t)(now - can_headlamp_override_last_tx_tick) >= CAN_PERIOD_DIGITAL_STATUS_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_HEADLAMP_OVERRIDE_ACTIVE,
                                     can_headlamp_override_enable);
          if (status == HAL_OK)
          {
            can_headlamp_override_last_tx_tick = now;
            can_headlamp_override_tx_count++;
          }
        }
        break;

      default:
        if ((uint32_t)(now - can_busoff_signal_last_tx_tick) >= CAN_PERIOD_CAN_DIAGNOSTIC_SIGNAL_MS)
        {
          status = CAN1_SendU8Signal(CAN_ID_F407_CAN_BUSOFF, can_busoff_state);
          if (status == HAL_OK)
          {
            can_busoff_signal_last_tx_tick = now;
            can_busoff_signal_tx_count++;
          }
        }
        break;
    }

    can_signal_scheduler_index++;
    if (can_signal_scheduler_index >= 15U)
    {
      can_signal_scheduler_index = 0U;
    }

    if ((status == HAL_OK) || (status == HAL_BUSY))
    {
      return;
    }
  }
}

static void CAN1_UpdateDiagnostics(void)
{
  can_tx_mailbox_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
  can_last_error = HAL_CAN_GetError(&hcan1);
  can_esr_register = hcan1.Instance->ESR;

  can_last_error_code = (uint8_t)((can_esr_register >> 4U) & 0x07U);
  can_tx_error_counter = (uint8_t)((can_esr_register >> 16U) & 0xFFU);
  can_rx_error_counter = (uint8_t)((can_esr_register >> 24U) & 0xFFU);
  can_busoff_state = ((can_esr_register & CAN_ESR_BOFF) != 0U) ? 1U : 0U;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int32_t difference;
  int32_t direction;
  int32_t next_position;

  if (htim->Instance != TIM6)
  {
    return;
  }

  if (actuator_output_enable == 0U)
  {
    return;
  }

  difference = actuator_target_position - actuator_absolute_position;
  if ((difference >= -ACTUATOR_DEADBAND_STEP) &&
      (difference <= ACTUATOR_DEADBAND_STEP))
  {
    return;
  }

  direction = (difference > 0) ? 1 : -1;
  next_position = actuator_absolute_position + direction;

  if ((next_position < 0) || (next_position > ACTUATOR_MAX_STEP))
  {
    actuator_limit_count++;
    return;
  }

  actuator_absolute_position = next_position;

  if (direction > 0)
  {
    actuator_step_index++;
    if (actuator_step_index >= ACTUATOR_STEP_SEQUENCE_COUNT)
    {
      actuator_step_index = 0U;
    }
  }
  else
  {
    if (actuator_step_index == 0U)
    {
      actuator_step_index = ACTUATOR_STEP_SEQUENCE_COUNT - 1U;
    }
    else
    {
      actuator_step_index--;
    }
  }

  Actuator_WriteStep(actuator_step_index);
  actuator_step_count++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
  uint8_t index;
  uint32_t now;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    rx_header = (CAN_RxHeaderTypeDef){0};
    for (index = 0U; index < 8U; index++)
    {
      rx_data[index] = 0U;
    }

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      can_last_error = HAL_CAN_GetError(hcan);
      can_error_callback_count++;
      return;
    }

    can_rx_count++;
    can_last_rx_dlc = (uint8_t)rx_header.DLC;
    for (index = 0U; index < 8U; index++)
    {
      can_last_rx_data[index] = rx_data[index];
    }

    if (rx_header.IDE == CAN_ID_STD)
    {
      can_last_rx_id = rx_header.StdId;
    }
    else
    {
      can_last_rx_id = rx_header.ExtId;
      can_unknown_rx_count++;
      continue;
    }

    if ((rx_header.StdId == CAN_ID_H723_PING) &&
        (rx_header.RTR == CAN_RTR_DATA) &&
        (rx_header.DLC == 8U) &&
        (rx_data[0] == 0xA5U) &&
        (rx_data[1] == 0x5AU) &&
        (rx_data[2] == 0x02U) &&
        (rx_data[3] == 0x01U))
    {
      can_echo_data[0] = 0x5AU;
      can_echo_data[1] = 0xA5U;
      can_echo_data[2] = 0x02U;
      can_echo_data[3] = 0x02U;
      can_echo_data[4] = rx_data[4];
      can_echo_data[5] = rx_data[5];
      can_echo_data[6] = rx_data[6];
      can_echo_data[7] = rx_data[7];
      can_echo_pending = 1U;
      can_ping_rx_count++;
      continue;
    }

    now = HAL_GetTick();

    switch (rx_header.StdId)
    {
      case CAN_ID_H723_STEERING_OVERRIDE:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U))
        {
          can_steering_override_enable = (rx_data[0] != 0U) ? 1U : 0U;
          can_steering_override_last_tick = now;
          can_steering_override_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      case CAN_ID_H723_ACTUATOR_ENABLE:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U))
        {
          can_actuator_enable_command = (rx_data[0] != 0U) ? 1U : 0U;
          can_actuator_enable_last_tick = now;
          can_actuator_enable_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      case CAN_ID_H723_ACTUATOR_TARGET:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 2U))
        {
          can_actuator_target_command =
            (uint16_t)((uint16_t)rx_data[0] | ((uint16_t)rx_data[1] << 8U));
          if (can_actuator_target_command > ACTUATOR_MAX_STEP)
          {
            can_actuator_target_command = ACTUATOR_MAX_STEP;
          }
          can_actuator_target_last_tick = now;
          can_actuator_target_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      case CAN_ID_H723_HEADLAMP_OVERRIDE:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U))
        {
          can_headlamp_override_enable = (rx_data[0] != 0U) ? 1U : 0U;
          can_headlamp_override_last_tick = now;
          can_headlamp_override_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      case CAN_ID_H723_HEADLAMP_COMMAND:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U))
        {
          can_headlamp_command = (rx_data[0] != 0U) ? 1U : 0U;
          can_headlamp_command_last_tick = now;
          can_headlamp_command_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;


      case CAN_ID_H723_TURN_OVERRIDE:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U))
        {
          can_turn_override_enable = (rx_data[0] != 0U) ? 1U : 0U;
          can_turn_override_last_tick = now;
          can_turn_override_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      case CAN_ID_H723_TURN_MODE:
        if ((rx_header.RTR == CAN_RTR_DATA) && (rx_header.DLC == 1U) && (rx_data[0] <= 2U))
        {
          can_turn_mode_command = rx_data[0];
          can_turn_mode_last_tick = now;
          can_turn_mode_rx_count++;
          can_command_rx_count++;
        }
        else
        {
          can_unknown_rx_count++;
        }
        break;

      default:
        can_unknown_rx_count++;
        break;
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t error;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  error = HAL_CAN_GetError(hcan);
  can_last_error = error;
  can_error_callback_count++;

  if ((error & HAL_CAN_ERROR_BOF) != 0U)
  {
    can_busoff_count++;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc_dma_error_count++;
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
  uint32_t now;
  uint32_t last_control_tick = 0U;
  uint32_t last_can_diagnostic_tick = 0U;
  uint32_t last_dht11_tick = 0U;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  DWT_DelayInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  now = HAL_GetTick();
  Switch_Init(&turn_switch_input, now);
  turn_switch_previous_pressed = turn_switch_input.stable_active;

  Actuator_SetEnabled(0U);

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)adc_dma_values,
                        ADC_VALUE_COUNT) != HAL_OK)
  {
    Error_Handler();
  }

  /* ADC values are read continuously; half/full transfer callbacks are not needed. */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);

  CAN1_ApplicationStart();

  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(20U);
  Application_UpdateInputsAndOutputs(HAL_GetTick());

  /* Treat the startup steering input as the synchronized initial position. */
  actuator_absolute_position = actuator_local_target_position;
  actuator_target_position = actuator_local_target_position;
  actuator_step_index = 0U;
  Actuator_WriteStep(actuator_step_index);
  Actuator_SetEnabled(actuator_local_enable);

  now = HAL_GetTick();
  last_control_tick = now;
  CAN1_SignalSchedulerInit(now);
  last_can_diagnostic_tick = now;
  last_dht11_tick = now - DHT11_READ_PERIOD_MS + 1000U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    CAN1_SendPendingEcho();

    if ((uint32_t)(now - last_control_tick) >= CONTROL_UPDATE_PERIOD_MS)
    {
      last_control_tick = now;
      Application_UpdateInputsAndOutputs(now);
    }

    CAN1_SendNextSignal(now);

    if ((uint32_t)(now - last_can_diagnostic_tick) >= CAN_DIAGNOSTIC_PERIOD_MS)
    {
      last_can_diagnostic_tick = now;
      CAN1_UpdateDiagnostics();
    }

    if ((uint32_t)(now - last_dht11_tick) >= DHT11_READ_PERIOD_MS)
    {
      last_dht11_tick = now;
      (void)DHT11_ReadSensor();
    }

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  Actuator_SetEnabled(0U);
  HAL_GPIO_WritePin(TURN_LEFT_LED_GPIO_Port, TURN_LEFT_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TURN_RIGHT_LED_GPIO_Port, TURN_RIGHT_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HEADLAMP_OUT_GPIO_Port, HEADLAMP_OUT_Pin, GPIO_PIN_RESET);
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
