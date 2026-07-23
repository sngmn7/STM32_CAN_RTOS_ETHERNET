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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//extern void test_imu_main(void); // 이 한줄만 테스트코드imu 쓸때만 살리고 아니면 꼭 지우기(임시 테스트 코드로 점프)
// 공유 변수 (태스크 간)
volatile int16_t g_m1_rpm_cmd = 0;
volatile int16_t g_m2_rpm_cmd = 0;
volatile int16_t g_m1_rpm_fb  = 0;
volatile int16_t g_m2_rpm_fb  = 0;

// UART 수신 버퍼
uint8_t jetson_rx_buf[6];
uint8_t jetson_rx_byte;

uint8_t rs485_rx_byte;
uint8_t rs485_ring[32];
uint8_t rs485_head = 0;

// Task handles
osThreadId_t motorTaskHandle;
osThreadId_t jetsonRxTaskHandle;
osThreadId_t jetsonTxTaskHandle;
osThreadId_t encoderTaskHandle;

const osThreadAttr_t encoderTask_attr = {
    .name = "EncoderTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t motorTask_attr = {
    .name = "MotorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityHigh,
};
const osThreadAttr_t jetsonRxTask_attr = {
    .name = "JetsonRxTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
const osThreadAttr_t jetsonTxTask_attr = {
    .name = "JetsonTxTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void MD200T_DualDrive(int16_t m1_rpm, int16_t m2_rpm);
void JetsonSendFeedback(int16_t m1_fb, int16_t m2_fb);
void MotorTask(void *argument);
void JetsonRxTask(void *argument);
void JetsonTxTask(void *argument);
void EncoderTask(void *argument);
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //test_imu_main(); // 테스트코드imu 쓸때만 살리고 아니면 꼭 지우기(임시 테스트 코드로 점프)
  HAL_UART_Receive_IT(&huart3, &rs485_rx_byte, 1);

  HAL_Delay(3000);

  // UART1 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart1, &jetson_rx_byte, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(EncoderTask, NULL, &encoderTask_attr);

  motorTaskHandle    = osThreadNew(MotorTask,    NULL, &motorTask_attr);
  jetsonRxTaskHandle = osThreadNew(JetsonRxTask, NULL, &jetsonRxTask_attr);
  jetsonTxTaskHandle = osThreadNew(JetsonTxTask, NULL, &jetsonTxTask_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MD200T_DualDrive(int16_t m1_rpm, int16_t m2_rpm) {
    uint8_t frame[13];
    frame[0]  = 183;
    frame[1]  = 184;
    frame[2]  = 1;
    frame[3]  = 207;
    frame[4]  = 7;
    frame[5]  = 1;
    frame[6]  = (uint8_t)(m1_rpm & 0xFF);
    frame[7]  = (uint8_t)((m1_rpm >> 8) & 0xFF);
    frame[8]  = 1;
    frame[9]  = (uint8_t)(m2_rpm & 0xFF);
    frame[10] = (uint8_t)((m2_rpm >> 8) & 0xFF);
    frame[11] = 1;

    uint8_t sum = 0;
    for(int i = 0; i < 12; i++) sum += frame[i];
    frame[12] = (uint8_t)(256 - sum);

    HAL_UART_Transmit(&huart3, frame, 13, 100);
}
// ── 젯슨으로 피드백 송신 ──────────────────────────────
void JetsonSendFeedback(int16_t m1_fb, int16_t m2_fb) {
    uint8_t pkt[6];
    pkt[0] = 0xBB;
    pkt[1] = (uint8_t)(m1_fb & 0xFF);
    pkt[2] = (uint8_t)((m1_fb >> 8) & 0xFF);
    pkt[3] = (uint8_t)(m2_fb & 0xFF);
    pkt[4] = (uint8_t)((m2_fb >> 8) & 0xFF);
    pkt[5] = pkt[1] + pkt[2] + pkt[3] + pkt[4]; // checksum
    HAL_UART_Transmit(&huart1, pkt, 6, 100);
}

// ── Task 1: 모터 구동 ─────────────────────────────────
void MotorTask(void *argument) {
    osDelay(3000);

    // BC On 3번 반복
    for(int i = 0; i < 3; i++) {
        uint8_t bc_on[] = {183, 184, 1, 10, 1, 5, 128};
        HAL_UART_Transmit(&huart3, bc_on, 7, 100);
        osDelay(200);

        uint8_t bc_on2[] = {183, 184, 2, 10, 1, 5, 127};
        HAL_UART_Transmit(&huart3, bc_on2, 7, 100);
        osDelay(200);
    }

    for(;;) {
        MD200T_DualDrive(g_m1_rpm_cmd, g_m2_rpm_cmd);
        osDelay(100);
    }
}
// ── Task 2: 젯슨 수신 ────────────────────────────────
// 바이트 링버퍼
static uint8_t rx_ring[16];
static uint8_t rx_head = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        rx_ring[rx_head++ & 0x0F] = jetson_rx_byte;
        HAL_UART_Receive_IT(&huart1, &jetson_rx_byte, 1);
    }
    if(huart->Instance == USART3) {  // ← 추가
        rs485_ring[rs485_head++ & 0x1F] = rs485_rx_byte;
        HAL_UART_Receive_IT(&huart3, &rs485_rx_byte, 1);
    }
}

void JetsonRxTask(void *argument) {
    uint8_t tail = 0;
    uint8_t buf[6];
    uint8_t idx = 0;

    for(;;) {
        // 링버퍼에 새 데이터 있으면 처리
        while(tail != rx_head) {
            uint8_t b = rx_ring[tail++ & 0x0F];

            if(idx == 0 && b != 0xAA) continue; // 헤더 아니면 무시
            buf[idx++] = b;

            if(idx == 6) {
                // 체크섬 검증
                uint8_t sum = buf[1] + buf[2] + buf[3] + buf[4];
                if(sum == buf[5]) {
                    g_m1_rpm_cmd = (int16_t)(buf[1] | (buf[2] << 8));
                    g_m2_rpm_cmd = (int16_t)(buf[3] | (buf[4] << 8));
                }
                idx = 0;
            }
        }
        osDelay(10);
    }
}

// ── Task 3: 젯슨 송신 ────────────────────────────────
void JetsonTxTask(void *argument) {
    for(;;) {
        JetsonSendFeedback(g_m1_rpm_fb, g_m2_rpm_fb); // 실제 엔코더값으로 변경
    	//JetsonSendFeedback(rs485_head, 0);
        osDelay(100);
    }
}
void EncoderTask(void *argument) {
    uint8_t tail = 0;
    uint8_t buf[20];
    uint8_t idx = 0;

    for(;;) {
        while(tail != rs485_head) {
            uint8_t b = rs485_ring[tail++ & 0x1F];

            if(idx == 0 && b != 184) { continue; }
            if(idx == 1 && b != 183) { idx = 0; continue; }
            buf[idx++] = b;

            if(idx == 20) {   // 헤더4 + 길이1 + 데이터14 + 체크섬1 = 20
                if(buf[3] == 216) {
                    g_m1_rpm_fb = (int16_t)(buf[5] | (buf[6] << 8));    // 모터1 RPM
                    g_m2_rpm_fb = (int16_t)(buf[12] | (buf[13] << 8));  // 모터2 RPM
                }
                idx = 0;
            }
        }
        osDelay(5);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	  for(;;)
	  {
	    osDelay(1000); // 그냥 대기만
	  }
  /* USER CODE END 5 */
}

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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
