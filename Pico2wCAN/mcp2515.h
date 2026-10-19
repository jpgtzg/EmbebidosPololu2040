#ifndef MCP2515_H
#define MCP2515_H

// Driver minimo del controlador CAN MCP2515 (datasheet Microchip DS20001801).
// Este archivo es IDENTICO en los dos proyectos (Zumo 2040 y Pico 2 W).
// Lo unico que cambia entre placas (pines, cristal, IDs) vive en config.h.

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "config.h"

// ---------------------------------------------------------------------------
// Instrucciones SPI (datasheet, tabla 12-1)
// ---------------------------------------------------------------------------
#define MCP2515_CMD_RESET        0xC0
#define MCP2515_CMD_READ         0x03
#define MCP2515_CMD_READ_RXB0    0x90   // READ RX BUFFER desde RXB0SIDH (limpia RX0IF al soltar CS)
#define MCP2515_CMD_READ_RXB1    0x94   // READ RX BUFFER desde RXB1SIDH (limpia RX1IF al soltar CS)
#define MCP2515_CMD_WRITE        0x02
#define MCP2515_CMD_LOAD_TXB0    0x40   // LOAD TX BUFFER desde TXB0SIDH
#define MCP2515_CMD_RTS_TXB0     0x81   // Request-To-Send del buffer TXB0
#define MCP2515_CMD_READ_STATUS  0xA0
#define MCP2515_CMD_BIT_MODIFY   0x05

// ---------------------------------------------------------------------------
// Registros
// ---------------------------------------------------------------------------
#define MCP2515_CANSTAT   0x0E
#define MCP2515_CANCTRL   0x0F
#define MCP2515_TEC       0x1C   // Transmit Error Counter
#define MCP2515_REC       0x1D   // Receive Error Counter
#define MCP2515_CNF3      0x28
#define MCP2515_CNF2      0x29
#define MCP2515_CNF1      0x2A
#define MCP2515_CANINTE   0x2B
#define MCP2515_CANINTF   0x2C
#define MCP2515_EFLG      0x2D

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
#define MCP2515_RXB1CTRL  0x70

// Bits de CANINTF / CANINTE
#define MCP2515_CANINTF_RX0IF 0x01
#define MCP2515_CANINTF_RX1IF 0x02
#define MCP2515_CANINTF_TX0IF 0x04
#define MCP2515_CANINTF_ERRIF 0x20
#define MCP2515_CANINTF_MERRF 0x80

// Bits de TXB0CTRL
#define MCP2515_TXB_ABTF  0x40   // transmision abortada
#define MCP2515_TXB_MLOA  0x20   // perdio arbitraje
#define MCP2515_TXB_TXERR 0x10   // error de bus al transmitir (tipicamente: nadie dio ACK)
#define MCP2515_TXB_TXREQ 0x08   // transmision pendiente

// Bits de EFLG
#define MCP2515_EFLG_RX1OVR 0x80
#define MCP2515_EFLG_RX0OVR 0x40
#define MCP2515_EFLG_TXBO   0x20   // bus-off
#define MCP2515_EFLG_TXEP   0x10   // TX error-passive (TEC >= 128)
#define MCP2515_EFLG_RXEP   0x08   // RX error-passive (REC >= 128)

// Modos de operacion (bits 7:5 de CANCTRL / CANSTAT)
#define MCP2515_MODE_NORMAL     0x00
#define MCP2515_MODE_SLEEP      0x20
#define MCP2515_MODE_LOOPBACK   0x40
#define MCP2515_MODE_LISTENONLY 0x60
#define MCP2515_MODE_CONFIG     0x80
#define MCP2515_MODE_MASK       0xE0

// Valor de CANCTRL justo despues de un RESET (datasheet, registro 10-1)
#define MCP2515_CANCTRL_RESET_VALUE 0x87

typedef struct {
    uint32_t id;        // 11 bits (estandar) o 29 bits (extendido)
    bool     extended;  // false = ID estandar de 11 bits (lo que pide la practica)
    uint8_t  dlc;       // 0..8 bytes de datos
    uint8_t  data[8];
} can_frame_t;

// Configura SPI + GPIO, resetea el chip y verifica que conteste por SPI.
// Regresa false si el MCP2515 no responde (cableado, alimentacion, pines).
bool    mcp2515_init(void);
void    mcp2515_reset(void);
bool    mcp2515_probe(void);

uint8_t mcp2515_read_register(uint8_t address);
void    mcp2515_write_register(uint8_t address, uint8_t value);
void    mcp2515_modify_register(uint8_t address, uint8_t mask, uint8_t value);
uint8_t mcp2515_read_status(void);

bool    mcp2515_set_mode(uint8_t mode);
uint8_t mcp2515_get_mode(void);
const char *mcp2515_mode_name(uint8_t mode);

// 500 kbit/s para un cristal de 8 MHz o de 16 MHz (lo que diga el cristal plateado del modulo)
bool    mcp2515_set_bitrate_500kbps(uint8_t crystal_mhz);

// Manda por TXB0 y espera a que algun nodo de ACK. Si nadie confirma en
// MCP2515_TX_TIMEOUT_MS, aborta la trama y regresa false.
bool    mcp2515_send_message(const can_frame_t *frame);
uint8_t mcp2515_last_tx_flags(void);   // TXB0CTRL capturado en el ultimo fallo de TX

bool    mcp2515_message_available(void);
bool    mcp2515_read_message(can_frame_t *frame);   // lee RXB0 o RXB1

void    mcp2515_print_diagnostics(void);

#endif // MCP2515_H
