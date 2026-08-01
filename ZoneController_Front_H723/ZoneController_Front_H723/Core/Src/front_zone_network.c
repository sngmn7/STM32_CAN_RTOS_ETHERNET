#include "front_zone_network.h"
#include "zone_udp_protocol.h"
#include "main.h"

#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

/* Values received from the Front F407 over CAN. */
extern volatile uint16_t f407_brake_adc;
extern volatile uint8_t f407_turn_left_output;
extern volatile uint8_t f407_turn_right_output;
extern volatile uint8_t f407_can_busoff;
extern volatile uint32_t f407_brake_rx_count;
extern volatile uint32_t f407_turn_left_rx_count;
extern volatile uint32_t f407_turn_right_rx_count;

/* Commands transmitted by the Front H723 to the Front F407. */
extern volatile uint8_t front_steering_override_enable;
extern volatile uint8_t front_actuator_enable_command;
extern volatile uint16_t front_actuator_target_command;
extern volatile uint8_t front_headlamp_override_enable;
extern volatile uint8_t front_headlamp_command;
extern volatile uint8_t front_turn_override_enable;
extern volatile uint8_t front_turn_mode_command;

volatile uint8_t front_network_initialized = 0U;
volatile uint32_t front_interzone_tx_count = 0U;
volatile uint32_t front_interzone_tx_error_count = 0U;
volatile uint32_t front_rear_status_rx_count = 0U;
volatile uint32_t front_rear_status_bad_count = 0U;
volatile uint8_t front_rear_link_alive = 0U;
volatile uint8_t rear_applied_turn_mode = 0U;
volatile uint8_t rear_applied_brake = 0U;
volatile uint8_t rear_command_source_flags = 0U;

volatile uint8_t front_brake_recalibrate = 0U;
volatile uint8_t front_brake_calibrated = 0U;
volatile uint16_t front_brake_baseline = 0U;
volatile uint16_t front_brake_delta = 0U;
volatile uint16_t front_brake_on_delta = 300U;
volatile uint16_t front_brake_off_delta = 150U;
volatile uint8_t front_brake_pressed = 0U;

volatile uint8_t front_jetson_command_alive = 0U;
volatile uint32_t front_jetson_command_rx_count = 0U;
volatile uint32_t front_jetson_command_bad_count = 0U;

static struct udp_pcb *front_interzone_pcb = NULL;
static struct udp_pcb *front_command_pcb = NULL;
static ip_addr_t rear_ip_address;

static uint16_t front_state_sequence = 0U;
static uint32_t front_state_last_tx_tick = 0U;
static uint32_t rear_status_last_rx_tick = 0U;
static uint32_t front_jetson_last_rx_tick = 0U;
static uint32_t brake_last_rx_count = 0U;
static uint32_t brake_last_rx_tick = 0U;
static uint32_t turn_left_last_rx_count = 0U;
static uint32_t turn_right_last_rx_count = 0U;
static uint32_t turn_last_rx_tick = 0U;
static uint32_t brake_baseline_sum = 0U;
static uint16_t brake_baseline_samples = 0U;

static void FrontZone_SendAck(struct udp_pcb *pcb,
                              const ip_addr_t *address,
                              u16_t port,
                              const uint8_t *data,
                              uint16_t length)
{
    struct pbuf *reply = pbuf_alloc(PBUF_TRANSPORT, length, PBUF_RAM);

    if (reply == NULL)
    {
        return;
    }

    if (pbuf_take(reply, data, length) == ERR_OK)
    {
        (void)udp_sendto(pcb, reply, address, port);
    }

    pbuf_free(reply);
}

static void FrontZone_CommandCallback(void *arg,
                                      struct udp_pcb *pcb,
                                      struct pbuf *packet,
                                      const ip_addr_t *address,
                                      u16_t port)
{
    uint8_t data[ZONE_FRONT_COMMAND_LENGTH];
    uint8_t flags;
    uint16_t target;

    LWIP_UNUSED_ARG(arg);

    if (packet == NULL)
    {
        return;
    }

    if ((packet->tot_len != ZONE_FRONT_COMMAND_LENGTH) ||
        (pbuf_copy_partial(packet, data, sizeof(data), 0U) != sizeof(data)) ||
        (Zone_ValidatePacket(data,
                             sizeof(data),
                             ZONE_MAGIC_JETSON,
                             ZONE_MSG_FRONT_COMMAND) == 0U))
    {
        front_jetson_command_bad_count++;
        pbuf_free(packet);
        return;
    }

    flags = data[6];
    target = Zone_ReadU16LE(&data[7]);

    if ((data[9] > 2U) || (target > 4095U))
    {
        front_jetson_command_bad_count++;
        pbuf_free(packet);
        return;
    }

    front_steering_override_enable =
        ((flags & FRONT_CMD_FLAG_STEERING_OVERRIDE) != 0U) ? 1U : 0U;
    front_actuator_enable_command =
        ((flags & FRONT_CMD_FLAG_ACTUATOR_ENABLE) != 0U) ? 1U : 0U;
    front_headlamp_override_enable =
        ((flags & FRONT_CMD_FLAG_HEADLAMP_OVERRIDE) != 0U) ? 1U : 0U;
    front_headlamp_command =
        ((flags & FRONT_CMD_FLAG_HEADLAMP_ON) != 0U) ? 1U : 0U;
    front_turn_override_enable =
        ((flags & FRONT_CMD_FLAG_TURN_OVERRIDE) != 0U) ? 1U : 0U;
    front_actuator_target_command = target;
    front_turn_mode_command = data[9];

    front_jetson_last_rx_tick = HAL_GetTick();
    front_jetson_command_alive = 1U;
    front_jetson_command_rx_count++;

    FrontZone_SendAck(pcb, address, port, data, sizeof(data));
    pbuf_free(packet);
}

static void FrontZone_RearStatusCallback(void *arg,
                                         struct udp_pcb *pcb,
                                         struct pbuf *packet,
                                         const ip_addr_t *address,
                                         u16_t port)
{
    uint8_t data[ZONE_REAR_STATUS_LENGTH];

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(pcb);
    LWIP_UNUSED_ARG(address);
    LWIP_UNUSED_ARG(port);

    if (packet == NULL)
    {
        return;
    }

    if ((packet->tot_len == ZONE_REAR_STATUS_LENGTH) &&
        (pbuf_copy_partial(packet, data, sizeof(data), 0U) == sizeof(data)) &&
        (Zone_ValidatePacket(data,
                             sizeof(data),
                             ZONE_MAGIC_REAR,
                             ZONE_MSG_REAR_STATUS) != 0U) &&
        (data[6] <= 2U) &&
        (data[7] <= 1U))
    {
        rear_applied_turn_mode = data[6];
        rear_applied_brake = data[7];
        rear_command_source_flags = data[8];
        rear_status_last_rx_tick = HAL_GetTick();
        front_rear_link_alive = 1U;
        front_rear_status_rx_count++;
    }
    else
    {
        front_rear_status_bad_count++;
    }

    pbuf_free(packet);
}

static void FrontZone_ResetBrakeCalibration(void)
{
    front_brake_calibrated = 0U;
    front_brake_pressed = 0U;
    front_brake_baseline = 0U;
    front_brake_delta = 0U;
    brake_baseline_sum = 0U;
    brake_baseline_samples = 0U;
    front_brake_recalibrate = 0U;
}

static void FrontZone_UpdateBrake(uint32_t now)
{
    uint16_t raw;
    uint16_t delta;

    if (front_brake_recalibrate != 0U)
    {
        FrontZone_ResetBrakeCalibration();
    }

    if (f407_brake_rx_count == brake_last_rx_count)
    {
        if ((uint32_t)(now - brake_last_rx_tick) > FRONT_TO_REAR_TIMEOUT_MS)
        {
            front_brake_pressed = 0U;
        }
        return;
    }

    brake_last_rx_count = f407_brake_rx_count;
    brake_last_rx_tick = now;
    raw = f407_brake_adc;

    if (front_brake_calibrated == 0U)
    {
        brake_baseline_sum += raw;
        brake_baseline_samples++;

        if (brake_baseline_samples >= 50U)
        {
            front_brake_baseline =
                (uint16_t)(brake_baseline_sum / brake_baseline_samples);
            front_brake_calibrated = 1U;
        }
        return;
    }

    delta = (raw >= front_brake_baseline)
        ? (uint16_t)(raw - front_brake_baseline)
        : (uint16_t)(front_brake_baseline - raw);
    front_brake_delta = delta;

    if (front_brake_pressed == 0U)
    {
        if (delta >= front_brake_on_delta)
        {
            front_brake_pressed = 1U;
        }
    }
    else if (delta <= front_brake_off_delta)
    {
        front_brake_pressed = 0U;
    }
}

static uint8_t FrontZone_GetTurnMode(uint32_t now)
{
    if ((uint32_t)(now - turn_last_rx_tick) > FRONT_TO_REAR_TIMEOUT_MS)
    {
        return 0U;
    }

    if ((f407_turn_left_output != 0U) && (f407_turn_right_output == 0U))
    {
        return 1U;
    }

    if ((f407_turn_right_output != 0U) && (f407_turn_left_output == 0U))
    {
        return 2U;
    }

    return 0U;
}

static void FrontZone_SendState(uint32_t now)
{
    uint8_t data[ZONE_FRONT_STATE_LENGTH] = {0U};
    struct pbuf *packet;
    err_t result;
    uint8_t flags = 0U;

    data[0] = ZONE_MAGIC_0;
    data[1] = ZONE_MAGIC_FRONT;
    data[2] = ZONE_PROTOCOL_VERSION;
    data[3] = ZONE_MSG_FRONT_STATE;
    Zone_WriteU16LE(&data[4], front_state_sequence++);
    data[6] = FrontZone_GetTurnMode(now);
    data[7] = front_brake_pressed;
    Zone_WriteU16LE(&data[8], f407_brake_adc);

    if ((uint32_t)(now - brake_last_rx_tick) <= FRONT_TO_REAR_TIMEOUT_MS)
    {
        flags |= (1U << 0);
    }
    if ((uint32_t)(now - turn_last_rx_tick) <= FRONT_TO_REAR_TIMEOUT_MS)
    {
        flags |= (1U << 1);
    }
    if (f407_can_busoff != 0U)
    {
        flags |= (1U << 2);
    }
    if (front_brake_calibrated != 0U)
    {
        flags |= (1U << 3);
    }
    data[10] = flags;
    data[11] = Zone_Checksum(data, ZONE_FRONT_STATE_LENGTH - 1U);

    packet = pbuf_alloc(PBUF_TRANSPORT, sizeof(data), PBUF_RAM);
    if (packet == NULL)
    {
        front_interzone_tx_error_count++;
        return;
    }

    if (pbuf_take(packet, data, sizeof(data)) != ERR_OK)
    {
        pbuf_free(packet);
        front_interzone_tx_error_count++;
        return;
    }

    result = udp_send(front_interzone_pcb, packet);
    pbuf_free(packet);

    if (result == ERR_OK)
    {
        front_interzone_tx_count++;
    }
    else
    {
        front_interzone_tx_error_count++;
    }
}

err_t FrontZoneNetwork_Init(void)
{
    err_t result;

    if (front_network_initialized != 0U)
    {
        return ERR_OK;
    }

    front_command_pcb = udp_new();
    front_interzone_pcb = udp_new();
    if ((front_command_pcb == NULL) || (front_interzone_pcb == NULL))
    {
        return ERR_MEM;
    }

    result = udp_bind(front_command_pcb, IP_ADDR_ANY, FRONT_JETSON_COMMAND_PORT);
    if (result != ERR_OK)
    {
        return result;
    }
    udp_recv(front_command_pcb, FrontZone_CommandCallback, NULL);

    result = udp_bind(front_interzone_pcb, IP_ADDR_ANY, FRONT_INTERZONE_LOCAL_PORT);
    if (result != ERR_OK)
    {
        return result;
    }

    IP_ADDR4(&rear_ip_address, 172, 30, 1, 200);
    result = udp_connect(front_interzone_pcb, &rear_ip_address, REAR_INTERZONE_PORT);
    if (result != ERR_OK)
    {
        return result;
    }
    udp_recv(front_interzone_pcb, FrontZone_RearStatusCallback, NULL);

    FrontZone_ResetBrakeCalibration();
    front_state_last_tx_tick = HAL_GetTick() - FRONT_STATE_TX_PERIOD_MS;
    brake_last_rx_tick = HAL_GetTick();
    turn_last_rx_tick = HAL_GetTick();
    front_network_initialized = 1U;
    return ERR_OK;
}

void FrontZoneNetwork_Process(uint32_t now)
{
    FrontZone_UpdateBrake(now);

    if ((f407_turn_left_rx_count != turn_left_last_rx_count) ||
        (f407_turn_right_rx_count != turn_right_last_rx_count))
    {
        turn_left_last_rx_count = f407_turn_left_rx_count;
        turn_right_last_rx_count = f407_turn_right_rx_count;
        turn_last_rx_tick = now;
    }

    if ((front_jetson_command_alive != 0U) &&
        ((uint32_t)(now - front_jetson_last_rx_tick) > JETSON_COMMAND_TIMEOUT_MS))
    {
        front_jetson_command_alive = 0U;
        front_steering_override_enable = 0U;
        front_actuator_enable_command = 0U;
        front_headlamp_override_enable = 0U;
        front_headlamp_command = 0U;
        front_turn_override_enable = 0U;
        front_turn_mode_command = 0U;
    }

    if ((front_rear_link_alive != 0U) &&
        ((uint32_t)(now - rear_status_last_rx_tick) > REAR_STATUS_TIMEOUT_MS))
    {
        front_rear_link_alive = 0U;
    }

    if ((front_interzone_pcb != NULL) &&
        ((uint32_t)(now - front_state_last_tx_tick) >= FRONT_STATE_TX_PERIOD_MS))
    {
        front_state_last_tx_tick = now;
        FrontZone_SendState(now);
    }
}
