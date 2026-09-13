/**
 * @file    dtc_proto.h
 * @brief   고장 진단 코드(DTC) 정의 및 'SD' UDP 패킷 포맷
 *
 * STM32(Front/Rear)와 Jetson 로거가 같은 파일을 공유한다.
 * 한쪽만 고치면 크기/버전 검사에서 전량 거부되므로 반드시 동시 반영할 것.
 *
 *   Front H723 --UDP 5105 ('SD' zone='F')--> Jetson
 *   Rear  H723 --UDP 5105 ('SD' zone='R')--> Jetson
 *
 * 역할 분담
 *   STM32  : 감지 -> 디바운스 -> 확정 -> 저장 -> 조치(페일세이프) -> 보고
 *   Jetson : 수신 -> 이력 저장 -> 표시. 추가로 "STM32 무응답" 자체를 판정.
 *
 * STM32 가 판정 주체인 이유
 *   - CAN 버스오프는 FDCAN 레지스터를 읽어야 하므로 게이트웨이만 관측 가능
 *   - 네트워크가 끊기면 Jetson 은 원인을 구분할 수 없다
 *   - 페일세이프 조치가 STM32 에서 일어나므로 판정 주체가 같아야 일관됨
 */

#ifndef DTC_PROTO_H
#define DTC_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define DTC_PORT        5105U
#define DTC_MAGIC       0x4453U   /* 'SD' LE */
#define DTC_VERSION     1U

/** 한 존이 보고할 수 있는 최대 DTC 수 */
#define DTC_MAX_ENTRIES 12U

/** 프리즈 프레임 신호 슬롯 수 (존별 의미는 COMM_ICD.md 참조) */
#define DTC_FREEZE_SIGNALS 8U

/* ------------------------------------------------------------------ */
/* DTC 코드 — SAE J2012 2바이트 인코딩                                  */
/*                                                                     */
/*   bit15..14  00=P(파워트레인) 01=C(섀시) 10=B(바디) 11=U(네트워크)   */
/*   bit13..0   나머지 4자리 (16진)                                     */
/*                                                                     */
/*   예) U0100 -> 0b11 << 14 | 0x0100 = 0xC100                          */
/* ------------------------------------------------------------------ */

/* --- U: 네트워크/통신 --- */
#define DTC_U0001_CAN500_BUSOFF     0xC001U  /**< FDCAN1(500k) 버스오프        */
#define DTC_U0002_CAN250_BUSOFF     0xC002U  /**< FDCAN2(250k) 버스오프        */
#define DTC_U0003_NODE_BUSOFF       0xC003U  /**< 하위 노드가 버스오프를 보고  */
#define DTC_U0100_ZONE_LINK_LOST    0xC100U  /**< Front <-> Rear 존간 링크 상실 */
#define DTC_U0200_FRAME_INTEGRITY   0xC200U  /**< 프레임 무결성 오류 급증      */
#define DTC_U0300_HOST_LINK_LOST    0xC300U  /**< Jetson 명령 링크 상실        */

/* --- P: 센서 --- */
#define DTC_P0500_ULTRASONIC_LOST   0x0500U  /**< 초음파(0x312) 무응답         */
#define DTC_P0501_IMU_LOST          0x0501U  /**< IMU(0x301) 무응답            */
#define DTC_P0502_SWITCH_LOST       0x0502U  /**< 토글스위치(0x311) 무응답     */
#define DTC_P0503_DHT_INVALID       0x0503U  /**< DHT11 온습도 무효            */
#define DTC_P0510_NODE_SIGNAL_LOST  0x0510U  /**< 하위 노드 CAN 신호 두절      */

/* --- Jetson 이 판정하는 코드 (STM32 는 보내지 않음) --- */
#define DTC_U0101_FRONT_NO_RESPONSE 0xC101U  /**< Front H723 텔레메트리 두절   */
#define DTC_U0102_REAR_NO_RESPONSE  0xC102U  /**< Rear H723 텔레메트리 두절    */

/* ------------------------------------------------------------------ */
/* 상태 비트 — ISO 14229(UDS) statusOfDTC 축소판                        */
/* ------------------------------------------------------------------ */

#define DTC_STS_TEST_FAILED     (1U << 0)  /**< 지금 이 순간 고장 상태        */
#define DTC_STS_PENDING         (1U << 1)  /**< 감지됐으나 확정 전(디바운스 중)*/
#define DTC_STS_CONFIRMED       (1U << 2)  /**< 확정. 정상 복귀해도 이력 유지  */
#define DTC_STS_WARNING         (1U << 3)  /**< 운전자 경고 필요 (안전 관련)   */

/**
 * @brief DTC 한 건 — 16 바이트
 *
 * detect_ms 는 장치가 스스로 측정한 검출 지연이다.
 * (원시 고장 발생 -> 확정까지의 실측 시간, 최대 65535 ms 포화)
 * 호스트가 별도로 재지 않아도 되고, CAN 계열처럼 외부에서
 * 고장 발생 시각을 알 수 없는 경우에도 정확한 값을 얻는다.
 */
struct dtc_entry_wire {
	uint16_t code;        /*  0  SAE J2012 인코딩            */
	uint8_t  status;      /*  2  DTC_STS_*                   */
	uint8_t  occurrence;  /*  3  발생 횟수 (255 에서 포화)   */
	uint32_t first_ms;    /*  4  최초 확정 시각 (uptime)     */
	uint32_t last_ms;     /*  8  최근 고장 관측 시각         */
	uint16_t detect_ms;   /* 12  실측 검출 지연              */
	uint16_t reserved;    /* 14                              */
} __attribute__((packed));

#define DTC_ENTRY_WIRE_SIZE  16U

/**
 * @brief 프리즈 프레임 — 24 바이트
 *
 * OBD-II 와 같이 존당 하나만 유지한다. 최초로 확정된 DTC 시점의
 * 신호 스냅샷을 저장해 사후 분석에 쓴다.
 * Diag_Clear() 전까지 덮어쓰지 않는다 — 첫 고장이 근본 원인일
 * 가능성이 높고, 이후 연쇄 고장이 원본을 지우면 안 되기 때문.
 *
 * sig[] 의 의미는 존별로 다르며 COMM_ICD.md 에 정의한다.
 */
struct dtc_freeze_wire {
	uint16_t code;        /*  0  이 프레임을 트리거한 DTC    */
	uint8_t  valid;       /*  2  0 = 캡처된 적 없음          */
	uint8_t  reserved;    /*  3                              */
	uint32_t uptime_ms;   /*  4  캡처 시각                   */
	uint16_t sig[DTC_FREEZE_SIGNALS];  /* 8..23  신호 스냅샷 */
} __attribute__((packed));

#define DTC_FREEZE_WIRE_SIZE  24U

/**
 * @brief 진단 패킷 — 232 바이트
 *
 * 상태 변화 시 즉시 + 1초 하트비트로 전송한다.
 * 텔레메트리('SR'/'SV')와 분리한 이유:
 *   - 기존 패킷 크기를 바꾸면 Jetson 커널 드라이버가 전량 거부한다
 *   - 진단은 이벤트성이라 주기가 다르다
 *   - UDS 조회 인터페이스로 확장하기 쉽다
 */
struct dtc_packet {
	uint16_t magic;       /*  0  'SD'                        */
	uint8_t  version;     /*  2                              */
	uint8_t  zone;        /*  3  'F'=0x46 / 'R'=0x52         */
	uint32_t seq;         /*  4                              */
	uint32_t uptime_ms;   /*  8                              */
	uint8_t  count;       /* 12  유효 엔트리 수              */
	uint8_t  active;      /* 13  현재 TEST_FAILED 인 개수    */
	uint8_t  confirmed;   /* 14  CONFIRMED 이력 개수         */
	uint8_t  reserved;    /* 15                              */

	struct dtc_entry_wire  entries[DTC_MAX_ENTRIES]; /* 16..207 */
	struct dtc_freeze_wire freeze;                   /* 208..231 */
} __attribute__((packed));

#define DTC_PACKET_SIZE  232U

typedef char dtc_packet_size_check
	[(sizeof(struct dtc_packet) == DTC_PACKET_SIZE) ? 1 : -1];

/* ------------------------------------------------------------------ */
/* 표시용 문자열 변환 (호스트 측 편의)                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief 0xC100 -> "U0100" 형태로 변환
 * @param buf 최소 6바이트
 */
static inline void DTC_ToString(uint16_t code, char *buf)
{
	static const char prefix[4] = { 'P', 'C', 'B', 'U' };
	static const char hex[16] = "0123456789ABCDEF";

	buf[0] = prefix[(code >> 14) & 0x3U];
	buf[1] = hex[(code >> 12) & 0x3U];
	buf[2] = hex[(code >> 8) & 0xFU];
	buf[3] = hex[(code >> 4) & 0xFU];
	buf[4] = hex[code & 0xFU];
	buf[5] = '\0';
}

#ifdef __cplusplus
}
#endif

#endif /* DTC_PROTO_H */
