#include "main.h"

volatile uint32_t echo_time_us = 0;
volatile float distance_cm = 0.0f;
volatile float raw_distance_cm = 0.0f;
volatile int ultrasonic_state = 0;

/*
   HC-SR04 연결

   TRIG -> PA5
   ECHO -> PA6
   VCC  -> 5V
   GND  -> GND
*/

#define HCSR04_TRIG_PORT GPIOA
#define HCSR04_TRIG_PIN  GPIO_PIN_5

#define HCSR04_ECHO_PORT GPIOA
#define HCSR04_ECHO_PIN  GPIO_PIN_6

void DWT_Init(void);
void delay_us(uint32_t us);
float HCSR04_Read(void);
void test_imu_main(void);

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    ultrasonic_state = 1;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

float HCSR04_Read(void)
{
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t timeout = 0;

    ultrasonic_state = 10;

    // TRIG LOW
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT,
                      HCSR04_TRIG_PIN,
                      GPIO_PIN_RESET);

    delay_us(5);

    // TRIG HIGH 10us
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT,
                      HCSR04_TRIG_PIN,
                      GPIO_PIN_SET);

    delay_us(10);

    HAL_GPIO_WritePin(HCSR04_TRIG_PORT,
                      HCSR04_TRIG_PIN,
                      GPIO_PIN_RESET);

    // ECHO HIGH 기다림
    timeout = 30000;

    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT,
                            HCSR04_ECHO_PIN) == GPIO_PIN_RESET)
    {
        if (timeout-- == 0)
        {
            ultrasonic_state = -1;
            return -1.0f;
        }

        delay_us(1);
    }

    ultrasonic_state = 20;

    start_time = DWT->CYCCNT;

    // ECHO LOW 기다림
    timeout = 30000;

    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT,
                            HCSR04_ECHO_PIN) == GPIO_PIN_SET)
    {
        if (timeout-- == 0)
        {
            ultrasonic_state = -2;
            return -2.0f;
        }

        delay_us(1);
    }

    end_time = DWT->CYCCNT;

    echo_time_us =
        (end_time - start_time) /
        (SystemCoreClock / 1000000U);

    raw_distance_cm = echo_time_us / 58.0f;

    if (raw_distance_cm > 1.0f &&
        raw_distance_cm < 400.0f)
    {
        distance_cm = raw_distance_cm;
        ultrasonic_state = 30;
    }
    else
    {
        ultrasonic_state = -3;
    }

    return raw_distance_cm;
}

void test_imu_main(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    ultrasonic_state = 100;

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // PA5 = TRIG OUTPUT
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT,
                      HCSR04_TRIG_PIN,
                      GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = HCSR04_TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(HCSR04_TRIG_PORT,
                  &GPIO_InitStruct);

    // PA6 = ECHO INPUT
    GPIO_InitStruct.Pin = HCSR04_ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(HCSR04_ECHO_PORT,
                  &GPIO_InitStruct);

    DWT_Init();

    while (1)
    {
        HCSR04_Read();

        delay_us(200000);
    }
}
