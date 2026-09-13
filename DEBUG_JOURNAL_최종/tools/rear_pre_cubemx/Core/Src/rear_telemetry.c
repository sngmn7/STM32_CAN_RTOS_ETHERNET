/**
 * @file    rear_telemetry.c
 * @brief   Rear 존 'SR' 패킷 UDP 송신 구현 (메인 루프 전용, ISR 아님)
 */

#include "rear_telemetry.h"
#include "zstatus_proto.h"

#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "stm32h7xx_hal.h"

/* 주기 송신(ms). 값이 안 바뀌어도 살아있음을 알리는 하트비트다. */
#define RT_PERIOD_MS   100U

/**
 * 이산 상태가 바뀌면 주기를 기다리지 않고 즉시 보낸다.
 * 다만 이 간격보다 촘촘히는 보내지 않아 송신이 폭주하지 않게 막는다.
 */
#define RT_MIN_GAP_MS   10U

/* 직전 송신에 담았던 이산 상태. 변화 감지용. */
static uint16_t rt_prev_discrete = 0xFFFFU;

/**
 * @brief 이산 상태를 한 워드로 압축한다.
 *
 * 연속량(front_brake_adc, ultrasonic_mm, imu_raw, 각종 수신 누계)은
 * 넣지 않는다. 잡음과 카운터 증가로 매 루프 바뀌어서 넣으면 사실상
 * 주기 없는 송신이 된다. 연속량은 주기 전송으로, 이벤트성 신호는
 * 변화 즉시 전송으로 나눈다.
 */
static uint16_t rt_discrete_word(const struct rt_values *v)
{
	return (uint16_t)(
		((v->turn_mode & 0x3U)                  ) |
		((v->led_left          ? 1U : 0U) <<  2) |
		((v->led_right         ? 1U : 0U) <<  3) |
		((v->led_brake         ? 1U : 0U) <<  4) |
		((v->window            ? 1U : 0U) <<  5) |
		((v->front_link_alive  ? 1U : 0U) <<  6) |
		((v->jetson_link_alive ? 1U : 0U) <<  7) |
		((v->turn_src_jetson   ? 1U : 0U) <<  8) |
		((v->brake_src_jetson  ? 1U : 0U) <<  9) |
		((v->can500_error      ? 1U : 0U) << 10) |
		((v->can250_error      ? 1U : 0U) << 11) |
		((v->switch_pressed    ? 1U : 0U) << 12));
}

/** 센서 값이 "신선하다"고 볼 최대 경과 시간 */
#define RT_SENSOR_FRESH_MS   1000U

/** 부호 있는 16비트 LE 두 바이트를 int16 으로 */
static int16_t rt_read_i16le(const uint8_t *d)
{
	return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
}

/** ★ Jetson 이더넷 IP ★ */
#define RT_JETSON_IP0  172
#define RT_JETSON_IP1  30
#define RT_JETSON_IP2  1
#define RT_JETSON_IP3  73

static struct udp_pcb *rt_pcb = NULL;
static ip_addr_t       rt_peer;
static uint32_t        rt_seq = 0U;
static uint32_t        rt_last_tick = 0U;

volatile uint32_t rt_tx_count = 0U;
volatile uint32_t rt_tx_error_count = 0U;


err_t RearTelemetry_Init(void)
{
	if (rt_pcb != NULL) {
		return ERR_OK;
	}

	rt_pcb = udp_new();

	if (rt_pcb == NULL) {
		return ERR_MEM;
	}

	IP4_ADDR(&rt_peer, RT_JETSON_IP0, RT_JETSON_IP1,
		 RT_JETSON_IP2, RT_JETSON_IP3);

	rt_last_tick = HAL_GetTick();

	return ERR_OK;
}


void RearTelemetry_Process(const struct rt_values *v)
{
	struct rstatus_packet pkt;
	struct pbuf *p;
	uint32_t now;

	if ((rt_pcb == NULL) || (v == NULL)) {
		return;
	}

	now = HAL_GetTick();

	{
		uint16_t d   = rt_discrete_word(v);
		uint32_t age = (uint32_t)(now - rt_last_tick);

		if (d != rt_prev_discrete) {
			if (age < RT_MIN_GAP_MS) {
				return;
			}
		} else if (age < RT_PERIOD_MS) {
			return;
		}
		rt_prev_discrete = d;
	}

	memset(&pkt, 0, sizeof(pkt));
	pkt.magic     = RST_MAGIC;
	pkt.version   = RST_VERSION;
	pkt.source    =
		(v->front_link_alive  ? RST_SRC_FRONT_LINK   : 0U) |
		(v->turn_src_jetson   ? RST_SRC_TURN_JETSON  : 0U) |
		(v->brake_src_jetson  ? RST_SRC_BRAKE_JETSON : 0U) |
		(v->jetson_link_alive ? RST_SRC_JETSON_LINK  : 0U);
	pkt.seq       = rt_seq;
	pkt.uptime_ms = now;
	pkt.turn_mode = v->turn_mode;
	pkt.led_left  = v->led_left ? 1U : 0U;
	pkt.led_right = v->led_right ? 1U : 0U;
	pkt.led_brake = v->led_brake ? 1U : 0U;
	pkt.window    = v->window ? 1U : 0U;
	pkt.health    =
		(v->can500_error ? RST_HLT_CAN500_ERR : 0U) |
		(v->can250_error ? RST_HLT_CAN250_ERR : 0U);
	pkt.front_brake_adc = v->front_brake_adc;

	/* --- F466RE 센서 (v2) --- */
	pkt.ultrasonic_mm = v->ultrasonic_mm;

	if (v->imu_raw != NULL) {
		pkt.quat_i    = rt_read_i16le(&v->imu_raw[0]);
		pkt.quat_j    = rt_read_i16le(&v->imu_raw[2]);
		pkt.quat_k    = rt_read_i16le(&v->imu_raw[4]);
		pkt.quat_real = rt_read_i16le(&v->imu_raw[6]);
	}

	pkt.sensor =
		(v->switch_pressed ? RST_SEN_SWITCH : 0U) |
		(((uint32_t)(now - v->ultra_last_tick)  <= RT_SENSOR_FRESH_MS)
			? RST_SEN_ULTRA_FRESH  : 0U) |
		(((uint32_t)(now - v->imu_last_tick)    <= RT_SENSOR_FRESH_MS)
			? RST_SEN_IMU_FRESH    : 0U) |
		(((uint32_t)(now - v->switch_last_tick) <= RT_SENSOR_FRESH_MS)
			? RST_SEN_SWITCH_FRESH : 0U);

	pkt.can500_rx_count = v->can500_rx_count;
	pkt.can250_rx_count = v->can250_rx_count;
	pkt.cmd_rx_count    = (uint16_t)(v->cmd_rx_count    & 0xFFFFU);
	pkt.imu_rx_count    = (uint16_t)(v->imu_rx_count    & 0xFFFFU);
	pkt.ultra_rx_count  = (uint16_t)(v->ultra_rx_count  & 0xFFFFU);
	pkt.switch_rx_count = (uint16_t)(v->switch_rx_count & 0xFFFFU);

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);

	if (p == NULL) {
		rt_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		rt_tx_error_count++;
		return;
	}

	if (udp_sendto(rt_pcb, p, &rt_peer, RST_PORT) == ERR_OK) {
		rt_seq++;
		rt_last_tick = now;
		rt_tx_count++;
	} else {
		rt_tx_error_count++;
	}

	pbuf_free(p);
}


/* ------------------------------------------------------------------ */
/* 성능 통계('SP' 5104) / 진단('SD' 5105) 송신                          */
/*                                                                     */
/* 텔레메트리와 같은 PCB(rt_pcb)를 목적지 포트만 바꿔 재사용한다.        */
/* UDP PCB 를 새로 만들지 않으므로 MEMP_NUM_UDP_PCB 한도에 영향 없음.    */
/* ------------------------------------------------------------------ */

#include "perf_proto.h"
#include "perf.h"
#include "dtc_proto.h"
#include "diagnostic.h"

#define RT_PERF_PERIOD_MS   1000U

static uint32_t rt_perf_last_tick = 0U;

volatile uint32_t rt_perf_tx_count = 0U;
volatile uint32_t rt_perf_tx_error_count = 0U;
volatile uint32_t rt_dtc_tx_count = 0U;
volatile uint32_t rt_dtc_tx_error_count = 0U;

static void rt_copy_stat(struct perf_stat_wire *dst,
			 const volatile perf_stat_t *src)
{
	dst->last_us  = src->last_us;
	dst->min_us   = (src->count != 0U) ? src->min_us : 0U;
	dst->max_us   = src->max_us;
	dst->avg_us   = src->avg_us;
	dst->count    = src->count;
	dst->over_1ms = src->over_1ms;
}


void RearTelemetry_SendPerf(void)
{
	static struct perf_packet pkt;   /* 104 B — 스택 회피 */
	struct pbuf *p;
	uint32_t now;

	if (rt_pcb == NULL) {
		return;
	}

	now = HAL_GetTick();

	if ((uint32_t)(now - rt_perf_last_tick) < RT_PERF_PERIOD_MS) {
		return;
	}

	memset(&pkt, 0, sizeof(pkt));
	pkt.magic     = PERF_MAGIC;
	pkt.version   = PERF_VERSION;
	pkt.flags     = (perf_ready != 0U) ? PERF_FLAG_DWT_READY : 0U;
	pkt.uptime_ms = now;

	rt_copy_stat(&pkt.loop,       &perf_loop_period);
	rt_copy_stat(&pkt.lwip,       &perf_lwip_process);
	rt_copy_stat(&pkt.can_isr,    &perf_can_isr);
	rt_copy_stat(&pkt.can_to_udp, &perf_can_to_udp);

	p = pbuf_alloc(PBUF_TRANSPORT, sizeof(pkt), PBUF_RAM);

	if (p == NULL) {
		rt_perf_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		rt_perf_tx_error_count++;
		return;
	}

	if (udp_sendto(rt_pcb, p, &rt_peer, PERF_PORT) == ERR_OK) {
		rt_perf_last_tick = now;
		rt_perf_tx_count++;
	} else {
		rt_perf_tx_error_count++;
	}

	pbuf_free(p);
}


void RearTelemetry_SendDtc(uint32_t now)
{
	static struct dtc_packet pkt;   /* 232 B — 스택 회피 */
	struct pbuf *p;

	if (rt_pcb == NULL) {
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
		rt_dtc_tx_error_count++;
		return;
	}

	if (pbuf_take(p, &pkt, sizeof(pkt)) != ERR_OK) {
		pbuf_free(p);
		rt_dtc_tx_error_count++;
		return;
	}

	if (udp_sendto(rt_pcb, p, &rt_peer, DTC_PORT) == ERR_OK) {
		Diag_NotifySent(now);
		rt_dtc_tx_count++;
	} else {
		rt_dtc_tx_error_count++;
	}

	pbuf_free(p);
}
