/**
 * @file    perf_proto.h
 * @brief   성능 계측 통계 UDP 전송 포맷 ('SP')
 *
 * STM32 와 Jetson 로거가 같은 파일을 공유한다.
 * 한쪽만 고치면 크기 검사에서 전량 거부되므로 반드시 동시 반영할 것.
 *
 *   Front H723 --UDP 5104 ('SP' 104B)--> Jetson perf_logger.py --> CSV
 *
 * 기존 텔레메트리(5102, 'SV')와는 별개 패킷이다.
 * 커널 드라이버가 검사하는 크기/버전을 건드리지 않기 위해 분리했다.
 */

#ifndef PERF_PROTO_H
#define PERF_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define PERF_PORT       5104U
#define PERF_MAGIC      0x5053U   /* 'SP' LE */
#define PERF_VERSION    1U

/* flags */
#define PERF_FLAG_DWT_READY   (1U << 0)   /**< DWT 카운터 정상 동작 */

/** @brief 구간 하나의 통계 (전부 마이크로초, count/over_1ms 는 횟수) */
struct perf_stat_wire {
	uint32_t last_us;    /* 0  */
	uint32_t min_us;     /* 4  */
	uint32_t max_us;     /* 8  */
	uint32_t avg_us;     /* 12 */
	uint32_t count;      /* 16 */
	uint32_t over_1ms;   /* 20 */
} __attribute__((packed));

#define PERF_STAT_WIRE_SIZE  24U

/**
 * @brief 성능 통계 패킷 — 104 바이트
 *
 * 1 Hz 로 보낸다. 통계는 부팅 이후 누적이므로
 * 로거는 시계열로 쌓아두고 마지막 샘플만 봐도 되고,
 * max 가 언제 튀었는지 추적해도 된다.
 */
struct perf_packet {
	uint16_t magic;                    /* 0  'SP'            */
	uint8_t  version;                  /* 2                  */
	uint8_t  flags;                    /* 3  PERF_FLAG_*     */
	uint32_t uptime_ms;                /* 4                  */

	struct perf_stat_wire loop;        /* 8   메인 루프 주기 */
	struct perf_stat_wire lwip;        /* 32  lwIP 처리      */
	struct perf_stat_wire can_isr;     /* 56  CAN ISR        */
	struct perf_stat_wire can_to_udp;  /* 80  CAN->UDP 종단  */
} __attribute__((packed));

#define PERF_PACKET_SIZE  104U

typedef char perf_packet_size_check
	[(sizeof(struct perf_packet) == PERF_PACKET_SIZE) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* PERF_PROTO_H */
