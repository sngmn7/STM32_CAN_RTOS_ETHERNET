/**
 * @file    rear_telemetry.h
 * @brief   Rear 존 상태('SR' 48바이트 v2)를 Jetson 으로 UDP 송신
 *
 * Rear 는 액추에이터를 "내리는" 존이면서 동시에
 * F466RE 로부터 센서를 "받는" 존이다. 둘 다 담는다.
 *   - 하행: 램프/윈도우 명령 + 그 출처 + 링크/CAN 헬스
 *   - 상행: 초음파(0x312) / IMU 쿼터니언(0x301) / 토글스위치(0x311)
 *
 * 호출 규약
 *   1) MX_LWIP_Init() 이후 RearTelemetry_Init()
 *   2) 메인 루프에서 rt_values 채워 RearTelemetry_Process() 호출
 *      (매 루프 호출 가능 — 내부 100ms 주기 제한)
 */

#ifndef REAR_TELEMETRY_H
#define REAR_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lwip/err.h"

struct rt_values {
	uint8_t  turn_mode;          /* rear_applied_turn_mode */
	uint8_t  led_left;           /* led_left_command */
	uint8_t  led_right;          /* led_right_command */
	uint8_t  led_brake;          /* led_brake_command (applied) */
	uint8_t  window;             /* window_command */
	uint8_t  front_link_alive;   /* rear_front_link_alive */
	uint8_t  jetson_link_alive;  /* rear_jetson_command_alive */
	uint8_t  turn_src_jetson;    /* rear_turn_source_jetson */
	uint8_t  brake_src_jetson;   /* rear_brake_source_jetson */
	uint8_t  can500_error;       /* 0/1 */
	uint8_t  can250_error;       /* 0/1 */
	uint16_t front_brake_adc;    /* rear_front_brake_adc */
	uint32_t can500_rx_count;
	uint32_t can250_rx_count;
	uint32_t cmd_rx_count;       /* rear_jetson_command_rx_count */

	/* --- F466RE 에서 CAN 으로 올라온 센서 (v2) --- */
	uint16_t ultrasonic_mm;      /* CAN 0x312 후방 거리 mm      */
	const uint8_t *imu_raw;      /* CAN 0x301 8바이트, NULL 허용 */
	uint8_t  switch_pressed;     /* CAN 0x311 토글스위치 0/1    */

	/* 신선도 판정용 — 각 센서를 마지막으로 받은 시각(HAL_GetTick) */
	uint32_t ultra_last_tick;
	uint32_t imu_last_tick;
	uint32_t switch_last_tick;

	/* 수신 누계 (링크 증거, 하위 16비트만 전송) */
	uint32_t imu_rx_count;
	uint32_t ultra_rx_count;
	uint32_t switch_rx_count;
};

err_t RearTelemetry_Init(void);
void  RearTelemetry_Process(const struct rt_values *v);

extern volatile uint32_t rt_tx_count;
extern volatile uint32_t rt_tx_error_count;

#ifdef __cplusplus
}
#endif

#endif /* REAR_TELEMETRY_H */
