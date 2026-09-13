/**
 * @file    diagnostic.c
 * @brief   DTC 감지 · 디바운스 · 저장 구현
 */

#include "diagnostic.h"

#include <string.h>

/** 상태 변화가 없어도 이 주기로 재송신 (링크 복구 시 동기화용) */
#define DIAG_HEARTBEAT_MS   1000U

static const diag_def_t *diag_defs = 0;
static diag_state_t      diag_state[DTC_MAX_ENTRIES];
static uint8_t           diag_count = 0U;
static uint8_t           diag_zone = 'F';

static uint32_t diag_seq = 0U;
static uint32_t diag_last_send_ms = 0U;
static uint8_t  diag_dirty = 0U;      /* 상태가 바뀌어 즉시 송신 필요 */

static diag_freeze_fn        diag_freeze_cb = 0;
static struct dtc_freeze_wire diag_freeze;

volatile uint8_t  diag_active_count = 0U;
volatile uint8_t  diag_confirmed_count = 0U;
volatile uint32_t diag_transition_count = 0U;


void Diag_Init(const diag_def_t *defs, uint8_t count, uint8_t zone,
               diag_freeze_fn freeze)
{
	if ((defs == 0) || (count == 0U)) {
		return;
	}

	diag_defs = defs;
	diag_count = (count > DTC_MAX_ENTRIES) ? DTC_MAX_ENTRIES : count;
	diag_zone = zone;
	diag_freeze_cb = freeze;

	memset(diag_state, 0, sizeof(diag_state));
	memset(&diag_freeze, 0, sizeof(diag_freeze));

	diag_seq = 0U;
	diag_last_send_ms = 0U;
	diag_dirty = 1U;   /* 부팅 직후 한 번은 보낸다 */

	diag_active_count = 0U;
	diag_confirmed_count = 0U;
	diag_transition_count = 0U;
}


void Diag_Report(uint8_t index, uint8_t failed, uint32_t now)
{
	diag_state_t *s;
	uint8_t raw;

	if ((diag_defs == 0) || (index >= diag_count)) {
		return;
	}

	s = &diag_state[index];
	raw = (failed != 0U) ? 1U : 0U;

	/* 원시 상태가 바뀐 순간을 기록 — 디바운스 기준점 */
	if (raw != s->raw) {
		s->raw = raw;
		s->raw_change_ms = now;
	}

	if (raw != 0U) {
		s->last_ms = now;
	}
}


void Diag_Process(uint32_t now)
{
	uint8_t i;
	uint8_t active = 0U;
	uint8_t confirmed = 0U;

	if (diag_defs == 0) {
		return;
	}

	for (i = 0U; i < diag_count; i++) {
		const diag_def_t *d = &diag_defs[i];
		diag_state_t *s = &diag_state[i];
		uint32_t held = (uint32_t)(now - s->raw_change_ms);
		uint8_t old_status = s->status;

		if (s->raw != 0U) {
			/*
			 * 고장 지속 중.
			 *
			 * 디바운스는 CONFIRMED 여부와 무관하게 항상 적용한다.
			 * CONFIRMED 는 "한 번이라도 확정된 적 있음"을 나타내는
			 * 래치일 뿐이고, TEST_FAILED 는 매번 confirm_ms 를
			 * 채워야 선다. 이렇게 해야 재발생 시에도 검출 지연이
			 * 동일하게 측정된다.
			 */
			if ((s->status & DTC_STS_TEST_FAILED) == 0U) {
				if (held >= d->confirm_ms) {
					/* 확정 */
					s->status |= DTC_STS_CONFIRMED |
						     DTC_STS_TEST_FAILED;
					s->status &= (uint8_t)~DTC_STS_PENDING;

					if (d->warning != 0U) {
						s->status |= DTC_STS_WARNING;
					}
					if (s->occurrence < 255U) {
						s->occurrence++;
					}
					if (s->first_ms == 0U) {
						s->first_ms = now;
					}

					/*
					 * 실측 검출 지연 = 원시 고장 시작 -> 확정.
					 * 매 확정마다 갱신하므로 최근값이 남는다.
					 */
					s->detect_ms = (held > 65535U)
						? 65535U : (uint16_t)held;

					/*
					 * 프리즈 프레임은 최초 1회만 캡처한다.
					 * 이후 연쇄 고장이 원본을 지우면 근본 원인을
					 * 잃기 때문 (OBD-II 와 같은 정책).
					 */
					if ((diag_freeze.valid == 0U) &&
					    (diag_freeze_cb != 0)) {
						/*
						 * packed 구조체 멤버의 주소를 직접
						 * 넘기면 정렬이 보장되지 않는다.
						 * 정렬된 로컬에 받아 복사한다.
						 */
						uint16_t tmp[DTC_FREEZE_SIGNALS];

						memset(tmp, 0, sizeof(tmp));
						diag_freeze_cb(tmp);

						memcpy(diag_freeze.sig, tmp,
						       sizeof(tmp));
						diag_freeze.code      = d->code;
						diag_freeze.uptime_ms = now;
						diag_freeze.valid     = 1U;
					}
				} else {
					/* 디바운스 대기 */
					s->status |= DTC_STS_PENDING;
				}
			}
			/* 이미 TEST_FAILED 인 경우는 유지만 하면 된다 */
		} else {
			/* 정상 복귀 */
			s->status &= (uint8_t)~DTC_STS_PENDING;

			if (((s->status & DTC_STS_TEST_FAILED) != 0U) &&
			    (held >= d->heal_ms)) {
				/*
				 * TEST_FAILED 만 내린다.
				 * CONFIRMED 는 남겨 이력으로 유지 —
				 * 간헐 고장을 놓치지 않기 위함.
				 * 삭제는 Diag_Clear() 로만.
				 */
				s->status &= (uint8_t)~DTC_STS_TEST_FAILED;
				s->status &= (uint8_t)~DTC_STS_WARNING;
			}
		}

		if (s->status != old_status) {
			diag_dirty = 1U;
			diag_transition_count++;
		}

		if ((s->status & DTC_STS_TEST_FAILED) != 0U) {
			active++;
		}
		if ((s->status & DTC_STS_CONFIRMED) != 0U) {
			confirmed++;
		}
	}

	diag_active_count = active;
	diag_confirmed_count = confirmed;
}


uint8_t Diag_ShouldSend(uint32_t now)
{
	if (diag_defs == 0) {
		return 0U;
	}

	if (diag_dirty != 0U) {
		return 1U;
	}

	return ((uint32_t)(now - diag_last_send_ms) >= DIAG_HEARTBEAT_MS)
		? 1U : 0U;
}


uint16_t Diag_BuildPacket(struct dtc_packet *pkt, uint32_t now)
{
	uint8_t i;

	if ((pkt == 0) || (diag_defs == 0)) {
		return 0U;
	}

	memset(pkt, 0, sizeof(*pkt));

	pkt->magic     = DTC_MAGIC;
	pkt->version   = DTC_VERSION;
	pkt->zone      = diag_zone;
	pkt->seq       = diag_seq;
	pkt->uptime_ms = now;
	pkt->count     = diag_count;
	pkt->active    = diag_active_count;
	pkt->confirmed = diag_confirmed_count;

	for (i = 0U; i < diag_count; i++) {
		pkt->entries[i].code       = diag_defs[i].code;
		pkt->entries[i].status     = diag_state[i].status;
		pkt->entries[i].occurrence = diag_state[i].occurrence;
		pkt->entries[i].first_ms   = diag_state[i].first_ms;
		pkt->entries[i].last_ms    = diag_state[i].last_ms;
		pkt->entries[i].detect_ms  = diag_state[i].detect_ms;
	}

	pkt->freeze = diag_freeze;

	return DTC_PACKET_SIZE;
}


void Diag_NotifySent(uint32_t now)
{
	diag_seq++;
	diag_last_send_ms = now;
	diag_dirty = 0U;
}


void Diag_Clear(void)
{
	uint8_t i;

	for (i = 0U; i < DTC_MAX_ENTRIES; i++) {
		diag_state[i].status     = 0U;
		diag_state[i].occurrence = 0U;
		diag_state[i].first_ms   = 0U;
		diag_state[i].last_ms    = 0U;
		diag_state[i].detect_ms  = 0U;
		/* raw / raw_change_ms 는 유지 — 현재 물리 상태이므로 */
	}

	memset(&diag_freeze, 0, sizeof(diag_freeze));

	diag_active_count = 0U;
	diag_confirmed_count = 0U;
	diag_dirty = 1U;
}
