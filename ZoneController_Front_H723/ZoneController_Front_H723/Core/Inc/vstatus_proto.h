/**
 * @file    vstatus_proto.h
 * @brief   차량 상태 테이블 와이어 프로토콜 (STM32 ↔ Jetson 공통 ICD)
 *
 * ★ 이 파일은 세 곳에 완전히 동일하게 존재해야 한다 ★
 *     stm32/Core/Inc/vstatus_proto.h
 *     jetson/kernel/vstatus_proto.h
 *     jetson/user/vstatus_proto.h
 *
 * 설계
 *   CAN 프레임 해석은 STM32(H723)가 담당한다. H723은 이미 CAN ID별로
 *   파싱한 변수(f407_*)를 갖고 있으므로, 그 결과 테이블을 고정 크기
 *   구조체 하나로 묶어 UDP로 보낸다.
 *
 *   Jetson 커널 드라이버는 이 구조체를 검증만 하고 /dev/vehicle_status
 *   로 노출한다. 가변 파싱이 없어 커널 코드가 단순/안전해진다.
 *
 * 포트: 5102 = zone_udp_protocol.h 의 FRONT_JETSON_TELEMETRY_PORT.
 *       정의만 있고 미구현이던 포트를 이번에 실제로 구현하는 것.
 *
 * 바이트 순서: 리틀 엔디안 (Cortex-M7, ARM64 둘 다 LE)
 * 패킷 크기: 정확히 28 바이트. 다르면 드라이버가 버린다.
 */

#ifndef VSTATUS_PROTO_H
#define VSTATUS_PROTO_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** STM32 → Jetson 텔레메트리 포트 */
#define VST_PORT        5102U

/** 'SV' 리틀 엔디안 */
#define VST_MAGIC       0x5653U

#define VST_VERSION     1U

/* flags 비트 */
#define VST_FLAG_DHT_VALID   (1U << 0)  /**< 온습도 값 유효 */
#define VST_FLAG_CAN_BUSOFF  (1U << 1)  /**< F407 CAN 버스오프 상태 */

/**
 * @brief 차량 상태 테이블 — 28 바이트 고정
 *
 * offset size  field
 *   0     2    magic          VST_MAGIC
 *   2     1    version        VST_VERSION
 *   3     1    flags          VST_FLAG_*
 *   4     4    seq            송신 시퀀스 (유실 감지)
 *   8     4    uptime_ms      H723 HAL_GetTick()
 *  12     2    steering_adc   0~4095
 *  14     2    accel_adc      0~4095
 *  16     2    brake_adc      0~4095
 *  18     2    actuator_pos   0~4095
 *  20     2    temp_x10       섭씨 x10 (부호 있음)
 *  22     2    humidity_x10   % x10
 *  24     1    turn_left      0/1
 *  25     1    turn_right     0/1
 *  26     1    headlamp       0/1
 *  27     1    reserved       0
 */
struct vstatus_packet {
	uint16_t magic;
	uint8_t  version;
	uint8_t  flags;
	uint32_t seq;
	uint32_t uptime_ms;
	uint16_t steering_adc;
	uint16_t accel_adc;
	uint16_t brake_adc;
	uint16_t actuator_pos;
	int16_t  temp_x10;
	uint16_t humidity_x10;
	uint8_t  turn_left;
	uint8_t  turn_right;
	uint8_t  headlamp;
	uint8_t  reserved;
} __attribute__((packed));

#define VST_PACKET_SIZE  28U

/* packed 누락/패딩 사고를 컴파일 타임에 잡는다 */
typedef char vstatus_packet_size_check
	[(sizeof(struct vstatus_packet) == VST_PACKET_SIZE) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* VSTATUS_PROTO_H */
