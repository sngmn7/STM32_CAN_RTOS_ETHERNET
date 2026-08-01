#ifndef REAR_ZONE_NETWORK_H
#define REAR_ZONE_NETWORK_H

#include <stdint.h>
#include "lwip/err.h"

#ifdef __cplusplus
extern "C" {
#endif

err_t RearZoneNetwork_Init(void);
void RearZoneNetwork_Process(uint32_t now);

extern volatile uint8_t rear_network_initialized;
extern volatile uint8_t rear_front_link_alive;
extern volatile uint8_t rear_jetson_command_alive;
extern volatile uint8_t rear_applied_turn_mode;
extern volatile uint8_t rear_applied_brake;
extern volatile uint8_t rear_turn_source_jetson;
extern volatile uint8_t rear_brake_source_jetson;
extern volatile uint32_t rear_front_state_rx_count;
extern volatile uint32_t rear_front_state_bad_count;
extern volatile uint32_t rear_jetson_command_rx_count;
extern volatile uint32_t rear_jetson_command_bad_count;
extern volatile uint32_t rear_status_tx_count;
extern volatile uint32_t rear_status_tx_error_count;
extern volatile uint16_t rear_front_brake_adc;

#ifdef __cplusplus
}
#endif

#endif /* REAR_ZONE_NETWORK_H */
