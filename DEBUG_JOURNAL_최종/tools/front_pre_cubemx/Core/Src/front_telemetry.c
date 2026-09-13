/**
 * @file    front_telemetry.c
 * @brief   차량 상태 테이블 UDP 송신 구현
 *
 * 메인 루프에서만 호출된다(ISR 아님) → lwIP raw API 를 바로 써도 안전.
 */

#include "front_telemetry.h"
#include "vstatus_proto.h"
#include "perf_proto.h"
#include "perf.h"
#include "dtc_proto.h"
#include "diagnostic.h"

#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* 설정                                                                */
/* ------------------------------------------------------------------ */

/** 주기 송신(ms). 값이 안 바뀌어도 살아있음을 알리는 하트비트다. */
#define FT_PERIOD_MS   100U

/**
 * 이산 상태가 바뀌면 주기를 기다리지 않고 즉시 보낸다.
 * 다만 이 간격보다 촘촘히는 보내지 않아, 값이 빠르게 흔들려도
 * 송신이 폭주하지 않게 막는다.
 */
#define FT_MIN_GAP_MS   10U

/** ★ Jetson 이더넷 IP. 실제 값으로 맞출 것 ★ */
#define FT_JETSON_IP0  172
#define FT_JETSON_IP1  30
#define FT_JETSON_IP2  1
#define FT_JETSON_IP3  73

/* ------------------------------------------------------------------ */

static struct udp_pcb *ft_pcb = NULL;
static ip_addr_t       ft_peer;
static uint32_t        ft_seq = 0U;
static uint32_t        ft_last_tick = 0U;

/* 직전 송신에 담았던 이산 상태. 변화 감지용. */
static uint8_t ft_prev_turn_left  = 0xFFU;
static uint8_t ft_prev_turn_right = 0xFFU;
static uint8_t ft_prev_headlamp   = 0xFFU;
static uint8_t ft_prev_busoff     = 0xFFU;
static uint8_t ft_prev_dht_valid  = 0xFFU;

volatile uint32_t ft_tx_count = 0U;
volatile uint32_t ft_tx_error_count = 0U;


err_t FrontTelemetry_Init(void)
{
	if (ft_pcb != NULL) {
		return ERR_OK;
	}

	ft_pcb = udp_new();

	if (ft_pcb == NULL) {
		return ERR_MEM;
	}

	/* 송신 전용 — bind 하지 않는다 */
	IP4_ADDR(&ft_peer, FT_JETSON_IP0, FT_JETSON_IP1,
		 FT_JETSON_IP2, FT_JETSON_IP3);

	ft_last_tick = HAL_GetTick();

	return ERR_OK;
}


void FrontTelemetry_Process(const struct ft_values *v)
{
	struct vstatus_packet pkt;
	struct pbuf *p;
	uint32_t now;
	uint32_t age;
	uint8_t  changed;

	if ((ft_pcb == NULL) || (v == NULL)) {
		return;
	}

	now = HAL_GetTick();
	age = (uint32_t)(now - ft_last_tick);

	/*
	 * 이산 상태만 변화 판정에 넣는다. ADC 값(조향/브레이크/가속/액추에이터
	 * 위치)은 잡음으로 매 루프 흔들려서 넣으면 사실상 매 바퀴 송신이 된다.
	 * 연속량은 주기 전송으로, 이벤트성 신호는 변화 즉시 전송으로 나눈다.
	 */
	changed = ((v->turn_left  ? 1U : 0U) != ft_prev_turn_left)  ||
		  ((v->turn_right ? 1U : 0U) != ft_prev_turn_right) ||
		  ((v->headlamp   ? 1U : 0U) != ft_prev_headlamp)   ||
		  ((v->busoff     ? 1U : 0U) != ft_prev_busoff)     ||
		  ((v->dht_valid  ? 1U : 0U) != ft_prev_dht_valid);

	if (changed) {
		if (age < FT_MIN_GAP_MS) {
			return;
		}
	} else if (age < FT_PERIOD_MS) {
		return;
	}

	ft_prev_turn_left  = v->turn_left  ? 1U : 0U;
	ft_prev_turn_right = v->turn_right ? 1U : 0U;
	ft_prev_headlamp   = v->headlamp   ? 1U : 0U;
	ft_prev_busoff     = v->busoff     ? 1U : 0U;
	ft_prev_dht_valid  = v->dht_valid  ? 1U : 0U;

	memset(&pkt, 0, sizeof(pkt));
	pkt.magic        = VST_MAGIC;
	pkt.version      = VST_VERSION;
	pkt.flags        = (v->dht_valid ? VST_FLAG_DHT_VALID : 0U) |
			   (v->busoff    ? VST_FLAG_CAN_BUSOFF : 0U);
	pkt.seq          = ft_seq;
	pkt.uptime_ms    = now;
	pkt.steering_adc = v->steering_adc;
	pkt.accel_adc    = v->accel_adc;
	pkt.brake_adc    = v->brake_adc;
	pkt.actuator_pos = v->actuator_pos;
	pkt.temp_x10     = v->temp_x10;
	pkt.humidity_x10 = v->humidity_x10;
	pkt.turn_left    = v->turn_left ? 1U : 0U;
	pkt.turn_right   = v->turn_right ? 1U : 0U;
	pkt.headlamp     = v->headlamp ? 1U : 0U;

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);

	if (p == NULL) {
		ft_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		ft_tx_error_count++;
		return;
	}

	if (udp_sendto(ft_pcb, p, &ft_peer, VST_PORT) == ERR_OK) {
		ft_seq++;
		ft_last_tick = now;
		ft_tx_count++;
	} else {
		ft_tx_error_count++;
	}

	pbuf_free(p);
}


/* ------------------------------------------------------------------ */
/* 성능 통계 송신 ('SP' 5104) — RTOS 전환 전후 비교용                   */
/* ------------------------------------------------------------------ */

#define FT_PERF_PERIOD_MS   1000U

static uint32_t ft_perf_last_tick = 0U;

volatile uint32_t ft_perf_tx_count = 0U;
volatile uint32_t ft_perf_tx_error_count = 0U;

/** perf_stat_t (volatile) -> 전송 구조체로 복사 */
static void ft_copy_stat(struct perf_stat_wire *dst,
			 const volatile perf_stat_t *src)
{
	dst->last_us  = src->last_us;
	/* 아직 샘플이 없으면 min 이 UINT32_MAX 이므로 0 으로 눕힌다 */
	dst->min_us   = (src->count != 0U) ? src->min_us : 0U;
	dst->max_us   = src->max_us;
	dst->avg_us   = src->avg_us;
	dst->count    = src->count;
	dst->over_1ms = src->over_1ms;
}


/**
 * @brief 성능 통계를 Jetson 으로 1 Hz 송신
 *
 * 텔레메트리와 같은 PCB(ft_pcb)를 쓰되 목적지 포트만 다르게 한다.
 * UDP PCB 를 새로 만들지 않으므로 MEMP_NUM_UDP_PCB 한도에 영향이 없다.
 * 메인 루프에서 매번 호출해도 되며 내부에서 주기를 제한한다.
 */
void FrontTelemetry_SendPerf(void)
{
	/* 232/104 B 를 스택에 올리면 1 KB 기본 스택을 넘긴다.
	 * 메인 루프 전용이라 재진입이 없으므로 static 으로 둔다. */
	static struct perf_packet pkt;
	struct pbuf *p;
	uint32_t now;

	if (ft_pcb == NULL) {
		return;
	}

	now = HAL_GetTick();

	if ((uint32_t)(now - ft_perf_last_tick) < FT_PERF_PERIOD_MS) {
		return;
	}

	memset(&pkt, 0, sizeof(pkt));
	pkt.magic     = PERF_MAGIC;
	pkt.version   = PERF_VERSION;
	pkt.flags     = (perf_ready != 0U) ? PERF_FLAG_DWT_READY : 0U;
	pkt.uptime_ms = now;

	ft_copy_stat(&pkt.loop,       &perf_loop_period);
	ft_copy_stat(&pkt.lwip,       &perf_lwip_process);
	ft_copy_stat(&pkt.can_isr,    &perf_can_isr);
	ft_copy_stat(&pkt.can_to_udp, &perf_can_to_udp);

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);

	if (p == NULL) {
		ft_perf_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		ft_perf_tx_error_count++;
		return;
	}

	if (udp_sendto(ft_pcb, p, &ft_peer, PERF_PORT) == ERR_OK) {
		ft_perf_last_tick = now;
		ft_perf_tx_count++;
	} else {
		ft_perf_tx_error_count++;
	}

	pbuf_free(p);
}


/* ------------------------------------------------------------------ */
/* 진단(DTC) 송신 ('SD' 5105)                                          */
/* ------------------------------------------------------------------ */

volatile uint32_t ft_dtc_tx_count = 0U;
volatile uint32_t ft_dtc_tx_error_count = 0U;

/**
 * @brief DTC 패킷을 Jetson 으로 송신
 *
 * 텔레메트리와 같은 PCB(ft_pcb)를 목적지 포트만 바꿔 재사용한다.
 * 송신 여부(상태 변화 or 하트비트)는 diagnostic 모듈이 판단하므로
 * 메인 루프에서 매번 호출해도 된다.
 */
void FrontTelemetry_SendDtc(uint32_t now)
{
	static struct dtc_packet pkt;   /* 232 B — 스택 회피 */
	struct pbuf *p;

	if (ft_pcb == NULL) {
		return;
	}

	if (Diag_ShouldSend(now) == 0U) {
		return;
	}

	if (Diag_BuildPacket(&pkt, now) != DTC_PACKET_SIZE) {
		return;
	}

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);

	if (p == NULL) {
		ft_dtc_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		ft_dtc_tx_error_count++;
		return;
	}

	if (udp_sendto(ft_pcb, p, &ft_peer, DTC_PORT) == ERR_OK) {
		Diag_NotifySent(now);
		ft_dtc_tx_count++;
	} else {
		ft_dtc_tx_error_count++;
	}

	pbuf_free(p);
}
