/**
 * @file    front_telemetry.h
 * @brief   해석된 차량 상태 테이블을 Jetson 으로 UDP 송신
 *
 * 흐름
 *   F407 --CAN--> H723 이 CAN ID 파싱 (기존 코드, f407_* 변수들)
 *                        │
 *                 FrontTelemetry_Process(&values)   ← 메인 루프에서
 *                        │  (내부에서 100ms 주기 제한)
 *                 UDP 5102 → Jetson /dev/vehicle_status
 *
 * 호출 규약
 *   1) MX_LWIP_Init() 이후 FrontTelemetry_Init()
 *   2) 메인 루프에서 ft_values 채워서 FrontTelemetry_Process() 호출
 *      (매 루프 호출해도 됨 — 내부에서 송신 주기를 조절한다)
 */

#ifndef FRONT_TELEMETRY_H
#define FRONT_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lwip/err.h"

/** 메인 루프가 채워서 넘기는 현재 상태 스냅샷 */
struct ft_values {
	uint16_t steering_adc;
	uint16_t accel_adc;
	uint16_t brake_adc;
	uint16_t actuator_pos;
	int16_t  temp_x10;
	uint16_t humidity_x10;
	uint8_t  dht_valid;
	uint8_t  turn_left;
	uint8_t  turn_right;
	uint8_t  headlamp;
	uint8_t  busoff;
};

err_t FrontTelemetry_Init(void);
void  FrontTelemetry_Process(const struct ft_values *v);

/* Live Expressions 관찰용 */
extern volatile uint32_t ft_tx_count;
extern volatile uint32_t ft_tx_error_count;

#ifdef __cplusplus
}
#endif

#endif /* FRONT_TELEMETRY_H */
