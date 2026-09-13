/**
 * @file    diagnostic.h
 * @brief   DTC 감지 · 디바운스 · 저장 (Front/Rear 공용)
 *
 * 사용법
 *   1) 존별 DTC 정의 테이블을 만들어 Diag_Init()
 *   2) 매 루프에서 원시 고장 여부를 Diag_Report(index, failed, now)
 *   3) Diag_Process(now) 가 디바운스/확정/치유 처리
 *   4) Diag_ShouldSend(now) 가 참이면 Diag_BuildPacket() 후 UDP 송신
 *
 * 디바운스가 필요한 이유
 *   link_alive 같은 플래그는 즉시 토글된다. 그대로 DTC 로 쓰면
 *   단발 패킷 손실에도 코드가 뜬다. "N ms 이상 지속" 조건을 붙여
 *   채터링을 막는다. 반대로 정상 복귀도 일정 시간 유지되어야 해제한다
 *   (히스테리시스).
 */

#ifndef DIAGNOSTIC_H
#define DIAGNOSTIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "dtc_proto.h"

/** @brief DTC 정의 — 컴파일 타임 상수 테이블 */
typedef struct {
	uint16_t code;         /**< DTC_* 코드                          */
	uint16_t confirm_ms;   /**< 이 시간 이상 고장 지속 시 확정       */
	uint16_t heal_ms;      /**< 이 시간 이상 정상 지속 시 해제       */
	uint8_t  warning;      /**< 1이면 운전자 경고 대상(안전 관련)    */
} diag_def_t;

/** @brief DTC 런타임 상태 */
typedef struct {
	uint8_t  raw;            /**< 최근 보고된 원시 고장 여부 0/1     */
	uint8_t  status;         /**< DTC_STS_*                          */
	uint8_t  occurrence;     /**< 발생 횟수 (255 포화)               */
	uint32_t raw_change_ms;  /**< raw 가 마지막으로 바뀐 시각        */
	uint32_t first_ms;       /**< 최초 확정 시각                     */
	uint32_t last_ms;        /**< 최근 고장 관측 시각                */
	uint16_t detect_ms;      /**< 실측 검출 지연 (raw 시작 -> 확정)  */
} diag_state_t;

/**
 * @brief 프리즈 프레임에 담을 신호를 채우는 콜백
 *
 * DTC 가 최초로 확정되는 순간 호출된다. 존별로 의미가 다른 신호를
 * sig[0..DTC_FREEZE_SIGNALS-1] 에 채워 넣으면 된다.
 * ISR 이 아닌 메인 루프에서 호출되므로 일반 변수를 그대로 읽어도 된다.
 */
typedef void (*diag_freeze_fn)(uint16_t *sig);

/**
 * @brief 진단 모듈 초기화
 * @param defs   DTC 정의 테이블 (static 수명이어야 함)
 * @param count  항목 수 (DTC_MAX_ENTRIES 이하)
 * @param zone   'F' 또는 'R'
 * @param freeze 프리즈 프레임 캡처 콜백 (NULL 허용)
 */
void Diag_Init(const diag_def_t *defs, uint8_t count, uint8_t zone,
               diag_freeze_fn freeze);

/**
 * @brief 원시 고장 여부 보고 (매 주기 호출)
 * @param index defs 배열 인덱스
 * @param failed 0=정상, 그 외=고장
 */
void Diag_Report(uint8_t index, uint8_t failed, uint32_t now);

/** @brief 디바운스/확정/치유 처리. 매 루프 호출. */
void Diag_Process(uint32_t now);

/**
 * @brief 송신해야 하는가
 *
 * 상태가 바뀌었거나 하트비트 주기(1초)가 됐으면 참.
 */
uint8_t Diag_ShouldSend(uint32_t now);

/**
 * @brief 전송용 패킷 생성
 * @return 채운 바이트 수 (항상 DTC_PACKET_SIZE)
 */
uint16_t Diag_BuildPacket(struct dtc_packet *pkt, uint32_t now);

/** @brief 송신 성공을 통보 — 하트비트 타이머 갱신 */
void Diag_NotifySent(uint32_t now);

/** @brief 확정 이력 삭제 (UDS 0x14 ClearDiagnosticInformation 대응) */
void Diag_Clear(void);

/* 디버거 관찰용 */
extern volatile uint8_t  diag_active_count;    /**< 현재 고장 중인 개수 */
extern volatile uint8_t  diag_confirmed_count; /**< 확정 이력 개수      */
extern volatile uint32_t diag_transition_count;/**< 상태 전이 누계      */

#ifdef __cplusplus
}
#endif

#endif /* DIAGNOSTIC_H */
