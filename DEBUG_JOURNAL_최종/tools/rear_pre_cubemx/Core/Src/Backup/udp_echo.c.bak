#include "udp_echo.h"

#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

static struct udp_pcb *udp_command_pcb = NULL;

static volatile uint8_t pending_command = 0U;
static volatile uint8_t pending_window = 0U;
static volatile uint8_t pending_left = 0U;
static volatile uint8_t pending_right = 0U;
static volatile uint8_t pending_brake = 0U;


/**
 * @brief Send the accepted 4-byte command back to Jetson as an ACK.
 */
static void UDP_SendCommandAck(
    struct udp_pcb *pcb,
    const ip_addr_t *remote_addr,
    u16_t remote_port,
    const uint8_t command[UDP_COMMAND_PACKET_LENGTH])
{
    struct pbuf *reply_pbuf;

    reply_pbuf = pbuf_alloc(
        PBUF_TRANSPORT,
        UDP_COMMAND_PACKET_LENGTH,
        PBUF_RAM);

    if (reply_pbuf == NULL)
    {
        return;
    }

    if (pbuf_take(
            reply_pbuf,
            command,
            UDP_COMMAND_PACKET_LENGTH) == ERR_OK)
    {
        (void)udp_sendto(
            pcb,
            reply_pbuf,
            remote_addr,
            remote_port);
    }

    pbuf_free(reply_pbuf);
}


/**
 * @brief LwIP UDP receive callback.
 */
static void UDP_CommandReceiveCallback(
    void *arg,
    struct udp_pcb *pcb,
    struct pbuf *received_pbuf,
    const ip_addr_t *remote_addr,
    u16_t remote_port)
{
    uint8_t command[UDP_COMMAND_PACKET_LENGTH];

    LWIP_UNUSED_ARG(arg);

    if (received_pbuf == NULL)
    {
        return;
    }

    /*
     * Accept only the fixed 4-byte actuator command packet.
     */
    if (received_pbuf->tot_len == UDP_COMMAND_PACKET_LENGTH)
    {
        if (pbuf_copy_partial(
                received_pbuf,
                command,
                UDP_COMMAND_PACKET_LENGTH,
                0U) == UDP_COMMAND_PACKET_LENGTH)
        {
            /*
             * Every actuator value must be exactly 0 or 1.
             */
            if ((command[0] <= 1U) &&
                (command[1] <= 1U) &&
                (command[2] <= 1U) &&
                (command[3] <= 1U))
            {
                pending_window = command[0];
                pending_left = command[1];
                pending_right = command[2];
                pending_brake = command[3];

                /*
                 * Set this flag last so main() never reads a half-written command.
                 */
                pending_command = 1U;

                UDP_SendCommandAck(
                    pcb,
                    remote_addr,
                    remote_port,
                    command);
            }
        }
    }

    pbuf_free(received_pbuf);
}


/**
 * @brief Create and bind the UDP command server.
 */
err_t UDP_Echo_Init(void)
{
    err_t result;

    if (udp_command_pcb != NULL)
    {
        return ERR_OK;
    }

    udp_command_pcb = udp_new();

    if (udp_command_pcb == NULL)
    {
        return ERR_MEM;
    }

    result = udp_bind(
        udp_command_pcb,
        IP_ADDR_ANY,
        UDP_COMMAND_PORT);

    if (result != ERR_OK)
    {
        udp_remove(udp_command_pcb);
        udp_command_pcb = NULL;
        return result;
    }

    udp_recv(
        udp_command_pcb,
        UDP_CommandReceiveCallback,
        NULL);

    return ERR_OK;
}


/**
 * @brief Return the newest accepted command to main() once.
 */
uint8_t UDP_Command_Take(
    uint8_t *window,
    uint8_t *left,
    uint8_t *right,
    uint8_t *brake)
{
    if ((window == NULL) ||
        (left == NULL) ||
        (right == NULL) ||
        (brake == NULL))
    {
        return 0U;
    }

    if (pending_command == 0U)
    {
        return 0U;
    }

    *window = pending_window;
    *left = pending_left;
    *right = pending_right;
    *brake = pending_brake;

    pending_command = 0U;

    return 1U;
}
