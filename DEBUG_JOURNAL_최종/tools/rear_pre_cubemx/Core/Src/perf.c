/**
 * @file    perf.c
 * @brief   DWT 사이클 카운터 기반 실행시간 계측 구현
 */

#include "perf.h"

volatile perf_stat_t perf_loop_period;
volatile perf_stat_t perf_lwip_process;
volatile perf_stat_t perf_can_isr;
volatile perf_stat_t perf_can_to_udp;

volatile uint8_t perf_ready = 0U;

/*
 * 주의 — DWT Lock Access Register(0xE0001FB0) 에 쓰지 말 것.
 *
 * 이 잠금 기구는 ARMv8-M(Cortex-M33 등)에만 존재하고 Cortex-M7 에는
 * 구현되어 있지 않다. 구현되지 않은 PPB 주소에 쓰면 버퍼된 쓰기가
 * 실패하고, 한참 뒤 버퍼가 드레인되는 시점에 IMPRECISERR
 * (CFSR bit10) HardFault 로 터진다.
 *
 * 부정확 폴트라 보고되는 PC 가 실제 원인 지점과 무관해서
 * (예: HAL_ETH_ReadData 로 보고됨) 추적이 매우 어렵다.
 *
 * Cortex-M7 에서는 CoreDebug->DEMCR 의 TRCENA 만 세우면 된다.
 */


void Perf_Reset(volatile perf_stat_t *s)
{
	s->last_us  = 0U;
	s->min_us   = 0xFFFFFFFFUL;
	s->max_us   = 0U;
	s->avg_us   = 0U;
	s->count    = 0U;
	s->over_1ms = 0U;
}


void Perf_ResetAll(void)
{
	Perf_Reset(&perf_loop_period);
	Perf_Reset(&perf_lwip_process);
	Perf_Reset(&perf_can_isr);
	Perf_Reset(&perf_can_to_udp);
}


void Perf_Init(void)
{
	Perf_ResetAll();

	/* 트레이스 유닛 활성화 — Cortex-M7 은 이것만으로 충분하다 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/*
	 * 실제로 증가하는지 확인한다. 디버거가 안 붙은 상태에서
	 * TRCENA 가 먹지 않는 경우가 있어 런타임 검증이 필요하다.
	 */
	{
		uint32_t a = DWT->CYCCNT;
		volatile uint32_t spin;

		for (spin = 0U; spin < 10U; spin++) {
			__NOP();
		}

		perf_ready = (DWT->CYCCNT != a) ? 1U : 0U;
	}
}


void Perf_Update(volatile perf_stat_t *s, uint32_t cycles)
{
	uint32_t us = cycles / PERF_CYC_PER_US;

	s->last_us = us;

	if (us < s->min_us) {
		s->min_us = us;
	}
	if (us > s->max_us) {
		s->max_us = us;
	}
	if (us > 1000U) {
		s->over_1ms++;
	}

	/*
	 * 1/16 IIR 이동평균.
	 *
	 * 주의: (us - avg) 를 unsigned 로 계산하면 us < avg 일 때
	 * 랩어라운드해서 평균이 통째로 깨진다. 반드시 부호 있는 연산.
	 */
	if (s->count == 0U) {
		s->avg_us = us;
	} else {
		int32_t diff = (int32_t)us - (int32_t)s->avg_us;

		s->avg_us = (uint32_t)((int32_t)s->avg_us + (diff / 16));
	}

	s->count++;
}
