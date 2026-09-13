#include "rear_zone_network.h"
#include "zone_udp_protocol.h"
#include "main.h"

#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

/* Existing command variables in Rear H723 main.c. */
extern volatile uint8_t led_left_command;
extern volatile uint8_t led_right_command;
extern volatile uint8_t led_brake_command;
extern volatile uint8_t window_command;
extern volatile FDCAN_ProtocolStatusTypeDef can250_protocol_status;

volatile uint8_t rear_network_initialized = 0U;
volatile uint8_t rear_front_link_alive = 0U;
volatile uint8_t rear_jetson_command_alive = 0U;
volatile uint8_t rear_applied_turn_mode = 0U;
volatile uint8_t rear_applied_brake = 0U;
volatile uint8_t rear_turn_source_jetson = 0U;
volatile uint8_t rear_brake_source_jetson = 0U;
volatile uint32_t rear_front_state_rx_count = 0U;
volatile uint32_t rear_front_state_bad_count = 0U;
volatile uint32_t rear_jetson_command_rx_count = 0U;
volatile uint32_t rear_jetson_command_bad_count = 0U;
volatile uint32_t rear_status_tx_count = 0U;
volatile uint32_t rear_status_tx_error_count = 0U;
volatile uint16_t rear_front_brake_adc = 0U;
volatile uint8_t rear_received_raw_turn_mode = 0U;

static struct udp_pcb *rear_command_pcb = NULL;
static struct udp_pcb *rear_interzone_pcb = NULL;

static uint8_t front_turn_mode = 0U;
static uint8_t front_brake_pressed = 0U;
static uint16_t front_sequence = 0U;
static uint32_t front_last_rx_tick = 0U;

static uint8_t jetson_override_flags = 0U;
static uint8_t jetson_turn_mode = 0U;
static uint8_t jetson_brake_command = 0U;
static uint8_t jetson_window_command = 0U;
static uint32_t jetson_last_rx_tick = 0U;

static ip_addr_t front_reply_address;
static u16_t front_reply_port = 0U;
static uint8_t rear_status_pending = 0U;
static uint32_t rear_status_last_tx_tick = 0U;

static void RearZone_SendAck(struct udp_pcb *pcb,
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

static void RearZone_CommandCallback(void *arg,
                                     struct udp_pcb *pcb,
                                     struct pbuf *packet,
                                     const ip_addr_t *address,
                                     u16_t port)
{
    uint8_t data[ZONE_REAR_COMMAND_LENGTH];
    uint8_t legacy[4];

    LWIP_UNUSED_ARG(arg);

    if (packet == NULL)
    {
        return;
    }

    /* Backward-compatible 4-byte command: window, left, right, brake. */
    if ((packet->tot_len == 4U) &&
        (pbuf_copy_partial(packet, legacy, sizeof(legacy), 0U) == sizeof(legacy)) &&
        (legacy[0] <= 1U) && (legacy[1] <= 1U) &&
        (legacy[2] <= 1U) && (legacy[3] <= 1U))
    {
        jetson_override_flags = REAR_CMD_FLAG_TURN_OVERRIDE |
                                 REAR_CMD_FLAG_BRAKE_OVERRIDE |
                                 REAR_CMD_FLAG_WINDOW_OVERRIDE;
        jetson_turn_mode = (legacy[1] != 0U) ? 1U :
                            ((legacy[2] != 0U) ? 2U : 0U);
        jetson_brake_command = legacy[3];
        jetson_window_command = legacy[0];
        jetson_last_rx_tick = HAL_GetTick();
        rear_jetson_command_alive = 1U;
        rear_jetson_command_rx_count++;
        RearZone_SendAck(pcb, address, port, legacy, sizeof(legacy));
        pbuf_free(packet);
        return;
    }

    if ((packet->tot_len != ZONE_REAR_COMMAND_LENGTH) ||
        (pbuf_copy_partial(packet, data, sizeof(data), 0U) != sizeof(data)) ||
        (Zone_ValidatePacket(data,
                             sizeof(data),
                             ZONE_MAGIC_JETSON,
                             ZONE_MSG_REAR_COMMAND) == 0U) ||
        (data[7] > 2U) || (data[8] > 1U) || (data[9] > 1U))
    {
        rear_jetson_command_bad_count++;
        pbuf_free(packet);
        return;
    }

    jetson_override_flags = data[6] &
        (REAR_CMD_FLAG_TURN_OVERRIDE |
         REAR_CMD_FLAG_BRAKE_OVERRIDE |
         REAR_CMD_FLAG_WINDOW_OVERRIDE);
    jetson_turn_mode = data[7];
    jetson_brake_command = data[8];
    jetson_window_command = data[9];
    jetson_last_rx_tick = HAL_GetTick();
    rear_jetson_command_alive = 1U;
    rear_jetson_command_rx_count++;

    RearZone_SendAck(pcb, address, port, data, sizeof(data));
    pbuf_free(packet);
}

static void RearZone_FrontStateCallback(void *arg,
                                        struct udp_pcb *pcb,
                                        struct pbuf *packet,
                                        const ip_addr_t *address,
                                        u16_t port)
{
    uint8_t data[ZONE_FRONT_STATE_LENGTH];

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(pcb);

    if (packet == NULL)
    {
        return;
    }

    if ((packet->tot_len == ZONE_FRONT_STATE_LENGTH) &&
        (pbuf_copy_partial(packet, data, sizeof(data), 0U) == sizeof(data)) &&
        (Zone_ValidatePacket(data,
                             sizeof(data),
                             ZONE_MAGIC_FRONT,
                             ZONE_MSG_FRONT_STATE) != 0U) &&
        (data[6] <= 2U) && (data[7] <= 1U))
    {
        front_sequence = Zone_ReadU16LE(&data[4]);
        rear_received_raw_turn_mode = data[6];
        front_turn_mode = data[6];
        front_brake_pressed = data[7];
        rear_front_brake_adc = Zone_ReadU16LE(&data[8]);
        front_last_rx_tick = HAL_GetTick();
        rear_front_link_alive = 1U;
        rear_front_state_rx_count++;
        ip_addr_copy(front_reply_address, *address);
        front_reply_port = port;
        rear_status_pending = 1U;
    }
    else
    {
        rear_front_state_bad_count++;
    }

    pbuf_free(packet);
}

static void RearZone_SendStatus(void)
{
    uint8_t data[ZONE_REAR_STATUS_LENGTH] = {0U};
    struct pbuf *packet;
    err_t result;
    uint8_t source_flags = 0U;

    if ((rear_interzone_pcb == NULL) || (front_reply_port == 0U))
    {
        return;
    }

    data[0] = ZONE_MAGIC_0;
    data[1] = ZONE_MAGIC_REAR;
    data[2] = ZONE_PROTOCOL_VERSION;
    data[3] = ZONE_MSG_REAR_STATUS;
    Zone_WriteU16LE(&data[4], front_sequence);
    data[6] = rear_applied_turn_mode;
    data[7] = rear_applied_brake;

    if (rear_front_link_alive != 0U) source_flags |= (1U << 0);
    if (rear_turn_source_jetson != 0U) source_flags |= (1U << 1);
    if (rear_brake_source_jetson != 0U) source_flags |= (1U << 2);
    if (rear_jetson_command_alive != 0U) source_flags |= (1U << 3);
    data[8] = source_flags;
    data[9] = (can250_protocol_status.BusOff != 0U) ? 1U : 0U;
    data[10] = window_command;
    data[11] = Zone_Checksum(data, ZONE_REAR_STATUS_LENGTH - 1U);

    packet = pbuf_alloc(PBUF_TRANSPORT, sizeof(data), PBUF_RAM);
    if (packet == NULL)
    {
        rear_status_tx_error_count++;
        return;
    }

    if (pbuf_take(packet, data, sizeof(data)) != ERR_OK)
    {
        pbuf_free(packet);
        rear_status_tx_error_count++;
        return;
    }

    result = udp_sendto(rear_interzone_pcb,
                        packet,
                        &front_reply_address,
                        front_reply_port);
    pbuf_free(packet);

    if (result == ERR_OK)
    {
        rear_status_tx_count++;
    }
    else
    {
        rear_status_tx_error_count++;
    }
}

err_t RearZoneNetwork_Init(void)
{
    err_t result;

    if (rear_network_initialized != 0U)
    {
        return ERR_OK;
    }

    rear_command_pcb = udp_new();
    rear_interzone_pcb = udp_new();
    if ((rear_command_pcb == NULL) || (rear_interzone_pcb == NULL))
    {
        return ERR_MEM;
    }

    result = udp_bind(rear_command_pcb, IP_ADDR_ANY, REAR_JETSON_COMMAND_PORT);
    if (result != ERR_OK)
    {
        return result;
    }
    udp_recv(rear_command_pcb, RearZone_CommandCallback, NULL);

    result = udp_bind(rear_interzone_pcb, IP_ADDR_ANY, REAR_INTERZONE_PORT);
    if (result != ERR_OK)
    {
        return result;
    }
    udp_recv(rear_interzone_pcb, RearZone_FrontStateCallback, NULL);

    rear_status_last_tx_tick = HAL_GetTick();
    rear_network_initialized = 1U;
    return ERR_OK;
}

void RearZoneNetwork_Process(uint32_t now)
{
    uint8_t jetson_valid;
    uint8_t front_valid;

    jetson_valid = ((rear_jetson_command_alive != 0U) &&
                     ((uint32_t)(now - jetson_last_rx_tick) <=
                      JETSON_COMMAND_TIMEOUT_MS)) ? 1U : 0U;
    front_valid = ((rear_front_link_alive != 0U) &&
                    ((uint32_t)(now - front_last_rx_tick) <=
                     FRONT_TO_REAR_TIMEOUT_MS)) ? 1U : 0U;

    if (jetson_valid == 0U)
    {
        rear_jetson_command_alive = 0U;
        jetson_override_flags = 0U;
    }
    if (front_valid == 0U)
    {
        rear_front_link_alive = 0U;
    }

    if ((jetson_valid != 0U) &&
        ((jetson_override_flags & REAR_CMD_FLAG_TURN_OVERRIDE) != 0U))
    {
        rear_applied_turn_mode = jetson_turn_mode;
        rear_turn_source_jetson = 1U;
    }
    else
    {
        rear_applied_turn_mode = (front_valid != 0U) ? front_turn_mode : 0U;
        rear_turn_source_jetson = 0U;
    }

    if ((jetson_valid != 0U) &&
        ((jetson_override_flags & REAR_CMD_FLAG_BRAKE_OVERRIDE) != 0U))
    {
        rear_applied_brake = jetson_brake_command;
        rear_brake_source_jetson = 1U;
    }
    else
    {
        rear_applied_brake = (front_valid != 0U) ? front_brake_pressed : 0U;
        rear_brake_source_jetson = 0U;
    }

    if ((jetson_valid != 0U) &&
        ((jetson_override_flags & REAR_CMD_FLAG_WINDOW_OVERRIDE) != 0U))
    {
        window_command = jetson_window_command;
    }
    else
    {
        window_command = 0U;
    }

    led_left_command = (rear_applied_turn_mode == 1U) ? 1U : 0U;
    led_right_command = (rear_applied_turn_mode == 2U) ? 1U : 0U;
    led_brake_command = rear_applied_brake;

    if ((rear_status_pending != 0U) ||
        ((front_reply_port != 0U) &&
         ((uint32_t)(now - rear_status_last_tx_tick) >= 100U)))
    {
        rear_status_pending = 0U;
        rear_status_last_tx_tick = now;
        RearZone_SendStatus();
    }
}
