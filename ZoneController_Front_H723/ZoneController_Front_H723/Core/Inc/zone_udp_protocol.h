#ifndef ZONE_UDP_PROTOCOL_H
#define ZONE_UDP_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZONE_PROTOCOL_VERSION                 1U

#define ZONE_MAGIC_0                          0x5AU
#define ZONE_MAGIC_FRONT                      0x46U
#define ZONE_MAGIC_REAR                       0x52U
#define ZONE_MAGIC_JETSON                     0x4AU

#define ZONE_MSG_FRONT_STATE                  0x10U
#define ZONE_MSG_REAR_STATUS                  0x11U
#define ZONE_MSG_FRONT_COMMAND                0x20U
#define ZONE_MSG_REAR_COMMAND                 0x21U

#define ZONE_FRONT_STATE_LENGTH               12U
#define ZONE_REAR_STATUS_LENGTH               12U
#define ZONE_FRONT_COMMAND_LENGTH             12U
#define ZONE_REAR_COMMAND_LENGTH              12U

#define FRONT_JETSON_COMMAND_PORT             5101U
#define FRONT_JETSON_TELEMETRY_PORT           5102U
#define FRONT_INTERZONE_LOCAL_PORT            5103U

#define REAR_JETSON_COMMAND_PORT              5001U
#define REAR_JETSON_TELEMETRY_PORT            5002U
#define REAR_INTERZONE_PORT                    5003U

#define FRONT_STATE_TX_PERIOD_MS              50U
#define FRONT_TO_REAR_TIMEOUT_MS              300U
#define REAR_STATUS_TIMEOUT_MS                300U
#define JETSON_COMMAND_TIMEOUT_MS             500U

/* Front Jetson command byte 6 flags. */
#define FRONT_CMD_FLAG_STEERING_OVERRIDE      (1U << 0)
#define FRONT_CMD_FLAG_ACTUATOR_ENABLE        (1U << 1)
#define FRONT_CMD_FLAG_HEADLAMP_OVERRIDE      (1U << 2)
#define FRONT_CMD_FLAG_HEADLAMP_ON            (1U << 3)
#define FRONT_CMD_FLAG_TURN_OVERRIDE          (1U << 4)

/* Rear Jetson command byte 6 flags. */
#define REAR_CMD_FLAG_TURN_OVERRIDE           (1U << 0)
#define REAR_CMD_FLAG_BRAKE_OVERRIDE          (1U << 1)
#define REAR_CMD_FLAG_WINDOW_OVERRIDE         (1U << 2)

static inline uint16_t Zone_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static inline void Zone_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static inline uint8_t Zone_Checksum(const uint8_t *data, uint16_t length_without_checksum)
{
    uint16_t index;
    uint8_t checksum = 0U;

    for (index = 0U; index < length_without_checksum; index++)
    {
        checksum ^= data[index];
    }

    return checksum;
}

static inline uint8_t Zone_ValidatePacket(const uint8_t *data,
                                          uint16_t length,
                                          uint8_t magic_1,
                                          uint8_t message_type)
{
    if ((data == (const uint8_t *)0) || (length < 5U))
    {
        return 0U;
    }

    if ((data[0] != ZONE_MAGIC_0) ||
        (data[1] != magic_1) ||
        (data[2] != ZONE_PROTOCOL_VERSION) ||
        (data[3] != message_type))
    {
        return 0U;
    }

    return (Zone_Checksum(data, (uint16_t)(length - 1U)) == data[length - 1U]) ? 1U : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* ZONE_UDP_PROTOCOL_H */
