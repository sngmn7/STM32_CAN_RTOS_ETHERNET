/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS 설정 — STM32H723ZG (Cortex-M7, 192 MHz)
 *
 * 이 프로젝트는 CubeMX 로 FreeRTOS 를 켜지 않고 손으로 통합했다.
 * 이유는 CubeMX 가 FreeRTOS 활성화 시 **lwIP 를 NO_SYS=0 으로 강제 전환**하기
 * 때문이다. 그러면 캐시 ECC / D2 SRAM 클럭 / MPU / PHY 타이밍까지 어렵게
 * 안정화한 이더넷 통합을 전부 다시 만들어야 한다.
 *
 *   -> lwIP 는 NO_SYS=1 을 유지하고, **단 하나의 태스크만** lwIP 를 만진다.
 *
 * 타임베이스도 CubeMX 기본(HAL=TIM6, FreeRTOS=SysTick)을 따르지 않았다.
 * 코드 전체가 HAL_GetTick() 기준으로 주기를 재고 있어서 의미를 바꾸고 싶지
 * 않았다. SysTick 하나를 둘이 쓰되, 스케줄러가 뜬 뒤에만 커널 틱을 돌린다.
 * (stm32h7xx_it.c 의 SysTick_Handler 참조)
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

extern uint32_t SystemCoreClock;

/* ------------------------------------------------------------------ */
/* 스케줄링                                                            */
/* ------------------------------------------------------------------ */
#define configUSE_PREEMPTION                     1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TIME_SLICING                   1
#define configUSE_NEWLIB_REENTRANT               0

/* ------------------------------------------------------------------ */
/* 동기화                                                              */
/* ------------------------------------------------------------------ */
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                8

/* ------------------------------------------------------------------ */
/* 메모리                                                              */
/*                                                                     */
/* heap_4 (병합 가능한 free list). 전부 부팅 때 한 번만 할당하고        */
/* 이후 동적 할당을 하지 않으므로 단편화 위험은 없다.                   */
/* RAM_D1 320 KB 중 현재 약 36 KB 사용 -> 32 KB 는 여유롭다.            */
/* ------------------------------------------------------------------ */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ((size_t)(32 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP         0

/* ------------------------------------------------------------------ */
/* 훅 — 조용히 실패하지 않게 전부 켠다                                  */
/* ------------------------------------------------------------------ */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           2   /* 패턴 검사까지 */
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0

/* ------------------------------------------------------------------ */
/* 런타임 통계 — 태스크별 CPU 점유 측정용                               */
/* DWT 사이클 카운터를 그대로 쓴다 (perf.c 가 이미 켜 둔다).            */
/* ------------------------------------------------------------------ */
#define configGENERATE_RUN_TIME_STATS            1
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     1

void     vConfigureTimerForRunTimeStats(void);
uint32_t vGetRunTimeCounterValue(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() vConfigureTimerForRunTimeStats()
#define portGET_RUN_TIME_COUNTER_VALUE()         vGetRunTimeCounterValue()

/* ------------------------------------------------------------------ */
/* 소프트웨어 타이머 — 지금은 안 쓴다                                   */
/* ------------------------------------------------------------------ */
#define configUSE_TIMERS                         0

/* ------------------------------------------------------------------ */
/* 포함할 API                                                          */
/* ------------------------------------------------------------------ */
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           0
#define INCLUDE_xTaskAbortDelay                  0
#define INCLUDE_xQueueGetMutexHolder             1
#define INCLUDE_xSemaphoreGetMutexHolder         1

/* ------------------------------------------------------------------ */
/* 인터럽트 우선순위                                                    */
/*                                                                     */
/* STM32 는 4비트 우선순위. 값이 클수록 낮은 우선순위다.                */
/* MAX_SYSCALL 보다 높은(숫자가 작은) 우선순위의 ISR 은 FreeRTOS API 를 */
/* 호출하면 안 된다 — 커널이 그 구간을 막지 못하기 때문이다.            */
/* ------------------------------------------------------------------ */
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* ------------------------------------------------------------------ */
/* 예외 핸들러 매핑                                                     */
/*                                                                     */
/* SysTick_Handler 는 매핑하지 않는다. HAL 도 SysTick 을 쓰기 때문에    */
/* stm32h7xx_it.c 에서 HAL_IncTick() 과 함께 호출한다.                  */
/* ------------------------------------------------------------------ */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler

/* ------------------------------------------------------------------ */
/* assert — 조용히 넘어가지 않게 잡아둔다                               */
/* ------------------------------------------------------------------ */
void vApplicationAssertCalled(const char *file, int line);
#define configASSERT(x) \
    do { if ((x) == 0) { vApplicationAssertCalled(__FILE__, __LINE__); } } while (0)

#endif /* FREERTOS_CONFIG_H */
