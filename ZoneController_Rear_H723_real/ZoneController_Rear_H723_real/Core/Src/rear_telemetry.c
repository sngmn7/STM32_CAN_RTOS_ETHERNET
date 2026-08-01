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

#define RT_PERIOD_MS   100U

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

	if ((uint32_t)(now - rt_last_tick) < RT_PERIOD_MS) {
		return;
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
