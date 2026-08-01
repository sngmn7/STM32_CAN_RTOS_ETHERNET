/**
 * @file    front_telemetry.c
 * @brief   차량 상태 테이블 UDP 송신 구현
 *
 * 메인 루프에서만 호출된다(ISR 아님) → lwIP raw API 를 바로 써도 안전.
 */

#include "front_telemetry.h"
#include "vstatus_proto.h"

#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "stm32h7xx_hal.h"

/* ------------------------------------------------------------------ */
/* 설정                                                                */
/* ------------------------------------------------------------------ */

/** 송신 주기(ms). 10Hz 면 LCD 표시용으로 충분하다. */
#define FT_PERIOD_MS   100U

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

	if ((ft_pcb == NULL) || (v == NULL)) {
		return;
	}

	now = HAL_GetTick();

	if ((uint32_t)(now - ft_last_tick) < FT_PERIOD_MS) {
		return;
	}

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
