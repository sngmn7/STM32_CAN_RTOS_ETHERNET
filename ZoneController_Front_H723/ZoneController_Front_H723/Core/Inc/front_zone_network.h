#ifndef FRONT_ZONE_NETWORK_H
#define FRONT_ZONE_NETWORK_H

#include <stdint.h>
#include "lwip/err.h"

#ifdef __cplusplus
extern "C" {
#endif

err_t FrontZoneNetwork_Init(void);
void FrontZoneNetwork_Process(uint32_t now);

/* Live Expressions / calibration. Do not press the brake during the first second. */
extern volatile uint8_t front_network_initialized;
extern volatile uint32_t front_interzone_tx_count;
extern volatile uint32_t front_interzone_tx_error_count;
extern volatile uint32_t front_rear_status_rx_count;
extern volatile uint32_t front_rear_status_bad_count;
extern volatile uint8_t front_rear_link_alive;
extern volatile uint8_t rear_applied_turn_mode;
extern volatile uint8_t rear_applied_brake;
extern volatile uint8_t rear_command_source_flags;

extern volatile uint8_t front_brake_recalibrate;
extern volatile uint8_t front_brake_calibrated;
extern volatile uint16_t front_brake_baseline;
extern volatile uint16_t front_brake_delta;
extern volatile uint16_t front_brake_on_delta;
extern volatile uint16_t front_brake_off_delta;
extern volatile uint8_t front_brake_pressed;

extern volatile uint8_t front_jetson_command_alive;
extern volatile uint32_t front_jetson_command_rx_count;
extern volatile uint32_t front_jetson_command_bad_count;

#ifdef __cplusplus
}
#endif

#endif /* FRONT_ZONE_NETWORK_H */
