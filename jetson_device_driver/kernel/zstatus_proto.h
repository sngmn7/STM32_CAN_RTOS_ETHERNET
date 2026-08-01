/**
 * @file    zstatus_proto.h
 * @brief   존 상태 텔레메트리 와이어 프로토콜 v2 — Front + Rear 공통 ICD
 *
 * ★ 이 파일은 네 곳에 완전히 동일하게 존재해야 한다 ★
 *     Front STM32:  Core/Inc/zstatus_proto.h
 *     Rear  STM32:  Core/Inc/zstatus_proto.h
 *     Jetson kernel: jetson/kernel/zstatus_proto.h
 *     Jetson user:   jetson/user/zstatus_proto.h
 *
 * 두 존은 서로 다른 패킷/포트를 쓴다:
 *
 *   Front: 'SV' 28바이트, UDP 5102 → /dev/vehicle_status
 *          F407 센서 15종의 해석 결과 (조향/가속/브레이크/온습도/등화...)
 *
 *   Rear : 'SR' 32바이트, UDP 5002 → /dev/rear_status
 *          Rear 는 CAN 으로 센서를 받지 않는다(핑 에코뿐).
 *          의미 있는 상태는 자기가 "내리고 있는" 액추에이터 명령과
 *          그 명령의 출처(Jetson vs Front 중재), 링크 생존, CAN 헬스다.
 *
 * 포트 5102/5002 = zone_udp_protocol.h 의 *_JETSON_TELEMETRY_PORT.
 * 바이트 순서: 리틀 엔디안.
 */

#ifndef ZSTATUS_PROTO_H
#define ZSTATUS_PROTO_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/* Front zone — 'SV' 28 bytes, UDP 5102                                */
/* ================================================================== */

#define VST_PORT        5102U
#define VST_MAGIC       0x5653U   /* 'SV' LE */
#define VST_VERSION     1U

/* flags 비트 — 0/1 값들을 모아 패킷 크기를 28바이트로 유지 */
#define VST_FLAG_DHT_VALID     (1U << 0)  /**< 0x219 온습도 값 유효 */
#define VST_FLAG_CAN_BUSOFF    (1U << 1)  /**< 0x21E F407 CAN 버스오프 */
#define VST_FLAG_TURN_SWITCH   (1U << 2)  /**< 0x215 방향지시 스위치 눌림 */
#define VST_FLAG_ACTUATOR_EN   (1U << 3)  /**< 0x21B 액추에이터 출력 허용 */
#define VST_FLAG_STEER_OVR     (1U << 4)  /**< 0x21C 조향 오버라이드 활성 */
#define VST_FLAG_HEADLAMP_OVR  (1U << 5)  /**< 0x21D 헤드램프 오버라이드 활성 */

struct vstatus_packet {
	uint16_t magic;          /* 0  */
	uint8_t  version;        /* 2  */
	uint8_t  flags;          /* 3  */
	uint32_t seq;            /* 4  */
	uint32_t uptime_ms;      /* 8  */
	uint16_t steering_adc;   /* 12 */
	uint16_t accel_adc;      /* 14 */
	uint16_t brake_adc;      /* 16 */
	uint16_t actuator_pos;   /* 18 */
	int16_t  temp_x10;       /* 20 */
	uint16_t humidity_x10;   /* 22 */
	uint8_t  turn_left;      /* 24 */
	uint8_t  turn_right;     /* 25 */
	uint8_t  headlamp;       /* 26 */
	uint8_t  reserved;       /* 27 */
} __attribute__((packed));

#define VST_PACKET_SIZE  28U

typedef char vstatus_packet_size_check
	[(sizeof(struct vstatus_packet) == VST_PACKET_SIZE) ? 1 : -1];

/* ================================================================== */
/* Rear zone — 'SR' 48 bytes, UDP 5002                                 */
/* ================================================================== */

#define RST_PORT        5002U
#define RST_MAGIC       0x5253U   /* 'SR' LE */
#define RST_VERSION     2U        /* v2: rear 센서(초음파/IMU/스위치) 추가 */

/* source 비트 — 명령 출처 중재와 링크 생존 */
#define RST_SRC_FRONT_LINK     (1U << 0)  /**< Front H723 링크 살아있음 */
#define RST_SRC_TURN_JETSON    (1U << 1)  /**< 방향지시 출처: Jetson(1)/Front(0) */
#define RST_SRC_BRAKE_JETSON   (1U << 2)  /**< 브레이크 출처: Jetson(1)/Front(0) */
#define RST_SRC_JETSON_LINK    (1U << 3)  /**< Jetson 명령 링크 살아있음 */

/* health 비트 — CAN 버스 상태 */
#define RST_HLT_CAN500_ERR     (1U << 0)  /**< FDCAN1(500k) 버스오프 */
#define RST_HLT_CAN250_ERR     (1U << 1)  /**< FDCAN2(250k) 버스오프 */

/* sensor 비트 — Rear 말단 노드에서 올라온 값의 상태 */
#define RST_SEN_SWITCH         (1U << 0)  /**< 토글 스위치 눌림 */
#define RST_SEN_ULTRA_FRESH    (1U << 1)  /**< 초음파 1초 내 수신 */
#define RST_SEN_IMU_FRESH      (1U << 2)  /**< IMU 1초 내 수신 */
#define RST_SEN_SWITCH_FRESH   (1U << 3)  /**< 스위치 1초 내 수신 */

/**
 * @brief Rear 존 상태 — 48 바이트
 *
 * Rear 는 액추에이터를 "내리는" 존이면서 동시에
 * F466RE 로부터 센서를 "받는" 존이다. 둘 다 담는다.
 */
struct rstatus_packet {
	uint16_t magic;             /* 0  'SR' */
	uint8_t  version;           /* 2  */
	uint8_t  source;            /* 3  RST_SRC_* */
	uint32_t seq;               /* 4  */
	uint32_t uptime_ms;         /* 8  */

	/* --- 내리고 있는 액추에이터 명령 --- */
	uint8_t  turn_mode;         /* 12 0 OFF, 1 LEFT, 2 RIGHT */
	uint8_t  led_left;          /* 13 CAN 0x320 */
	uint8_t  led_right;         /* 14 CAN 0x321 */
	uint8_t  led_brake;         /* 15 CAN 0x322 */
	uint8_t  window;            /* 16 CAN 0x323 */
	uint8_t  health;            /* 17 RST_HLT_* */
	uint16_t front_brake_adc;   /* 18 Front 가 중계해 준 값 */

	/* --- F466RE 에서 CAN 으로 올라온 센서 --- */
	uint16_t ultrasonic_mm;     /* 20 CAN 0x312, 후방 거리 mm */
	int16_t  quat_i;            /* 22 CAN 0x301, BNO08x 회전벡터 */
	int16_t  quat_j;            /* 24 */
	int16_t  quat_k;            /* 26 */
	int16_t  quat_real;         /* 28 */
	uint8_t  sensor;            /* 30 RST_SEN_* */
	uint8_t  reserved;          /* 31 */

	/* --- 카운터 (링크 증거) --- */
	uint32_t can500_rx_count;   /* 32 */
	uint32_t can250_rx_count;   /* 36 */
	uint16_t cmd_rx_count;      /* 40 Jetson 명령 수신 (하위 16비트) */
	uint16_t imu_rx_count;      /* 42 */
	uint16_t ultra_rx_count;    /* 44 */
	uint16_t switch_rx_count;   /* 46 */
} __attribute__((packed));

#define RST_PACKET_SIZE  48U

typedef char rstatus_packet_size_check
	[(sizeof(struct rstatus_packet) == RST_PACKET_SIZE) ? 1 : -1];

/* ------------------------------------------------------------------ */
/* Rear zone CAN ID 맵                                                 */
/* ------------------------------------------------------------------ */

/* FDCAN1 = 500 kbps : H723 <-> F407G */
#define RCAN_PING_500_TX        0x101U  /**< H723 -> F407G */
#define RCAN_PING_500_RX        0x181U  /**< F407G -> H723 에코 */

/* FDCAN2 = 250 kbps : H723 <-> F466RE */
#define RCAN_PING_250_TX        0x201U  /**< H723 -> F466RE */
#define RCAN_PING_250_RX        0x281U  /**< F466RE -> H723 에코 */

/* F466RE -> H723 센서 (상행) */
#define RCAN_IMU_QUAT           0x301U  /**< 4x int16 LE 회전벡터 */
#define RCAN_SWITCH_STATUS      0x311U  /**< data[0] = 눌림 0/1 */
#define RCAN_ULTRASONIC_MM      0x312U  /**< uint16 LE 거리 mm */

/* H723 -> F466RE 명령 (하행), 각 DLC 1 */
#define RCAN_LEFT_LAMP_CMD      0x320U
#define RCAN_RIGHT_LAMP_CMD     0x321U
#define RCAN_BRAKE_LAMP_CMD     0x322U
#define RCAN_WINDOW_CMD         0x323U

#ifdef __cplusplus
}
#endif

#endif /* ZSTATUS_PROTO_H */
