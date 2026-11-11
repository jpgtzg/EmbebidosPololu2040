#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"

#define MCP2515_SPI_PORT   spi0

#define MCP2515_PIN_MISO   0
#define MCP2515_PIN_CS     1
#define MCP2515_PIN_SCK    2
#define MCP2515_PIN_MOSI   3
#define MCP2515_PIN_INT    28

#define MCP2515_SPI_BAUD   1000000u  //u means unsigned

#define MCP2515_CMD_RESET       0xC0
#define MCP2515_CMD_READ        0x03
#define MCP2515_CMD_WRITE       0x02
#define MCP2515_CMD_RTS_TXB0    0x81
#define MCP2515_CMD_READ_STATUS 0xA0
#define MCP2515_CMD_BIT_MODIFY  0x05

#define MCP2515_CANSTAT   0x0E
#define MCP2515_CANCTRL   0x0F
#define MCP2515_CNF1      0x2A
#define MCP2515_CNF2      0x29
#define MCP2515_CNF3      0x28
#define MCP2515_CANINTE   0x2B
#define MCP2515_CANINTF   0x2C

#define MCP2515_TXB0CTRL  0x30
#define MCP2515_TXB0SIDH  0x31
#define MCP2515_TXB0SIDL  0x32
#define MCP2515_TXB0DLC   0x35
#define MCP2515_TXB0D0    0x36

#define MCP2515_RXB0CTRL  0x60
#define MCP2515_RXB0SIDH  0x61
#define MCP2515_RXB0SIDL  0x62
#define MCP2515_RXB0DLC   0x65
#define MCP2515_RXB0D0    0x66

#define MCP2515_CANINTF_RX0IF 0x01
#define MCP2515_CANINTF_TX0IF 0x04

// TXB0CTRL bit: transmission pending
#define MCP2515_TXB_TXREQ     0x08

#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80
#define MCP2515_MODE_MASK       0xE0

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} can_frame_t;

void    mcp2515_init(void);
void    mcp2515_reset(void);
uint8_t mcp2515_read_register(uint8_t address);
void    mcp2515_write_register(uint8_t address, uint8_t value);
void    mcp2515_modify_register(uint8_t address, uint8_t mask, uint8_t value);
uint8_t mcp2515_read_status(void);
bool    mcp2515_set_mode(uint8_t mode);
bool    mcp2515_set_bitrate_500kbps_8mhz(void);
bool    mcp2515_send_message(const can_frame_t *frame);
bool    mcp2515_message_available(void);
bool    mcp2515_read_message(can_frame_t *frame);

#endif // MCP2515_H
