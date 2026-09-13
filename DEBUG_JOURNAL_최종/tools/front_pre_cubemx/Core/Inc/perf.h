/**
 * @file    perf.h
 * @brief   DWT 사이클 카운터 기반 실행시간 계측 (RTOS 전환 전후 비교용)
 *
 * Cortex-M7 의 DWT->CYCCNT 를 그대로 읽는다. CPU 192 MHz 이므로
 * 1 us = 192 cycles, 분해능은 약 5.2 ns 다.
 *
 * 사용법
 *   1) 부팅 직후 Perf_Init()
 *   2) 구간 앞뒤로 PERF_NOW() 를 떠서 Perf_Update() 에 차이를 넘김
 *   3) 디버거 Live Expressions 에서 perf_* 전역을 관찰
 *
 * 오버헤드: PERF_NOW() 는 레지스터 1회 읽기, Perf_Update() 는
 * 비교 몇 번 + 곱셈 1회 수준이라 계측 대상 대비 무시할 만하다.
 */

#ifndef PERF_H
#define PERF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

/** CPU 코어 클럭 (Hz). SystemClock_Config 와 일치해야 한다. */
#define PERF_CPU_HZ        192000000UL

/** 사이클 -> 마이크로초. 상수 나눗셈이라 곱셈으로 컴파일된다. */
#define PERF_CYC_PER_US    (PERF_CPU_HZ / 1000000UL)   /* 192 */

/**
 * @brief 한 구간의 통계
 *
 * 값은 전부 마이크로초. min 은 첫 갱신 전 UINT32_MAX 로 두어
 * 최초 샘플이 그대로 들어가게 한다.
 */
typedef struct {
	uint32_t last_us;    /**< 최근 측정값            */
	uint32_t min_us;     /**< 최소                   */
	uint32_t max_us;     /**< 최대 (지터 판단 핵심)  */
	uint32_t avg_us;     /**< 이동 평균 (1/16 IIR)   */
	uint32_t count;      /**< 누적 샘플 수           */
	uint32_t over_1ms;   /**< 1 ms 초과 횟수         */
} perf_stat_t;

/* ------------------------------------------------------------------ */
/* 계측 대상                                                           */
/* ------------------------------------------------------------------ */

extern volatile perf_stat_t perf_loop_period;   /**< 메인 루프 1바퀴 주기      */
extern volatile perf_stat_t perf_lwip_process;  /**< MX_LWIP_Process() 소요    */
extern volatile perf_stat_t perf_can_isr;       /**< FDCAN RX 콜백 소요        */
extern volatile perf_stat_t perf_can_to_udp;    /**< CAN 수신 -> UDP 송신 지연 */

extern volatile uint8_t perf_ready;             /**< DWT 사용 가능 여부 */

/* ------------------------------------------------------------------ */

void Perf_Init(void);
void Perf_Update(volatile perf_stat_t *s, uint32_t cycles);
void Perf_Reset(volatile perf_stat_t *s);
void Perf_ResetAll(void);

/** @brief 현재 사이클 카운터. 계측 구간 앞뒤에서 호출한다. */
static inline uint32_t PERF_NOW(void)
{
	return DWT->CYCCNT;
}

/**
 * @brief 시작 시각부터 지금까지의 경과 사이클
 *
 * CYCCNT 는 32비트 랩어라운드지만 뺄셈이 unsigned 라
 * 한 바퀴(192 MHz 기준 약 22.4 초) 이내면 자동으로 올바른 값이 된다.
 */
static inline uint32_t PERF_ELAPSED(uint32_t start)
{
	return DWT->CYCCNT - start;
}

#ifdef __cplusplus
}
#endif

#endif /* PERF_H */
