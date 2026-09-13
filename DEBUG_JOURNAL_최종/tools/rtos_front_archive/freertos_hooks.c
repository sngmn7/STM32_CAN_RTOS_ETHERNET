/**
 * @file    freertos_hooks.c
 * @brief   FreeRTOS 훅 및 런타임 통계 소스
 *
 * 훅을 전부 켜 둔 이유는, RTOS 전환에서 가장 흔한 실패가 **조용히 잘못
 * 도는 것**이기 때문이다. 스택이 넘쳐도, 힙 할당이 실패해도 겉으로는
 * "그냥 좀 이상한 동작"으로만 보인다. 여기서 잡아 카운터에 남긴다.
 *
 * 디버거로 이 카운터들을 읽어 확인한다:
 *   rtos_stack_overflow_count / rtos_stack_overflow_task
 *   rtos_malloc_failed_count
 *   rtos_assert_count / rtos_assert_line
 */

#include "FreeRTOS.h"
#include "task.h"
#include "perf.h"

#include <string.h>

/* ------------------------------------------------------------------ */
/* 실패 기록                                                           */
/* ------------------------------------------------------------------ */
volatile uint32_t rtos_stack_overflow_count = 0U;
volatile char     rtos_stack_overflow_task[configMAX_TASK_NAME_LEN] = {0};
volatile uint32_t rtos_malloc_failed_count = 0U;
volatile uint32_t rtos_assert_count = 0U;
volatile uint32_t rtos_assert_line = 0U;
volatile const char *rtos_assert_file = 0;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;

    rtos_stack_overflow_count++;
    if (pcTaskName != 0) {
        strncpy((char *)rtos_stack_overflow_task, pcTaskName,
                configMAX_TASK_NAME_LEN - 1);
    }

    /* 스택이 이미 깨졌으므로 계속 돌리는 것이 더 위험하다. */
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationMallocFailedHook(void)
{
    rtos_malloc_failed_count++;

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationAssertCalled(const char *file, int line)
{
    rtos_assert_count++;
    rtos_assert_file = file;
    rtos_assert_line = (uint32_t)line;

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

/* ------------------------------------------------------------------ */
/* 런타임 통계용 시간원                                                 */
/*                                                                     */
/* DWT 사이클 카운터를 그대로 쓴다. perf.c 가 부팅 때 이미 켜 두므로    */
/* 여기서는 따로 설정할 것이 없다. 태스크별 CPU 점유율을 재는 데 쓰이고, */
/* 그 값이 "베어메탈 대비 오버헤드"를 보여주는 근거가 된다.             */
/*                                                                     */
/* 분해능은 5.2 ns 지만 32비트라 192 MHz 에서 약 22.4 초마다 랩어라운드  */
/* 한다. FreeRTOS 통계는 차이만 누적하므로 문제되지 않는다.             */
/* ------------------------------------------------------------------ */
void vConfigureTimerForRunTimeStats(void)
{
    /* perf.c 의 Perf_Init() 이 DWT->CYCCNT 를 이미 활성화한다. */
}

uint32_t vGetRunTimeCounterValue(void)
{
    return PERF_NOW();
}
