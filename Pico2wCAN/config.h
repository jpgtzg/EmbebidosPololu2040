#ifndef CONFIG_H
#define CONFIG_H

// ===========================================================================
//  NODO: Raspberry Pi Pico 2 W (RP2350)  ->  proyecto Pico2wCAN
//  Este es el UNICO archivo que cambia entre el proyecto del Zumo y el de la
//  Pico 2 W. mcp2515.c, mcp2515.h y main.c son identicos en ambos.
// ===========================================================================

#define NODE_NAME       "Pico 2 W"
#define BOARD_IS_ZUMO   0

// IDs de 11 bits (los de la practica): la Pico 2 W manda 0x200, el Zumo 0x500
#define CAN_TX_ID       0x200
#define CAN_PEER_ID     0x500

// Pines (tabla de la Parte 1 del PDF)
#define MCP2515_SPI_PORT   spi0
#define MCP2515_PIN_MISO   16   // SO del MCP2515  (pin fisico 21)
#define MCP2515_PIN_CS     17   // CS              (pin fisico 22)
#define MCP2515_PIN_SCK    18   // SCK             (pin fisico 24)
#define MCP2515_PIN_MOSI   19   // SI              (pin fisico 25)
#define MCP2515_PIN_INT    20   // INT             (pin fisico 26)
#define MCP2515_SPI_BAUD   (1000 * 1000)   // 1 MHz: seguro con cables de protoboard

// Frecuencia del cristal plateado del modulo MCP2515 (dice "8.000" o "16.000").
// Se puede cambiar en vivo con la tecla 'x' en el monitor serial.
#define MCP2515_CRYSTAL_MHZ 8

// 1 = arranca en NORMAL (Parte 4: platica con el otro nodo)
// 0 = arranca en LOOPBACK (Partes 2 y 3: se recibe a si mismo)
// Se puede cambiar en vivo con las teclas 'n' / 'l'.
#define CAN_START_IN_NORMAL_MODE 1

#endif // CONFIG_H
