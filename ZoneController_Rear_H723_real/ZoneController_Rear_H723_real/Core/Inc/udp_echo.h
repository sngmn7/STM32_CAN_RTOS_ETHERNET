#ifndef UDP_ECHO_H
#define UDP_ECHO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "lwip/err.h"

/*
 * UDP command packet, 4 bytes:
 *   byte 0: window command (0=OFF, 1=ON)
 *   byte 1: left indicator (0=OFF, 1=ON)
 *   byte 2: right indicator (0=OFF, 1=ON)
 *   byte 3: brake lamp (0=OFF, 1=ON)
 */
#define UDP_COMMAND_PORT           5001U
#define UDP_COMMAND_PACKET_LENGTH  4U

/**
 * @brief Start the UDP command server on port 5001.
 *
 * A valid 4-byte command is echoed back to the sender as an ACK.
 */
err_t UDP_Echo_Init(void);

/**
 * @brief Read and clear the newest pending UDP command.
 *
 * @return 1 when a new valid command was returned, otherwise 0.
 */
uint8_t UDP_Command_Take(
    uint8_t *window,
    uint8_t *left,
    uint8_t *right,
    uint8_t *brake);

#ifdef __cplusplus
}
#endif

#endif /* UDP_ECHO_H */
