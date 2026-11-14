#ifndef CONFIG_H
#define CONFIG_H

// ===========================================================================
//  NODO: Pololu Zumo 2040 (RP2040)  ->  proyecto ZumoCAN
//  Este es el UNICO archivo que cambia respecto a Pico2wCAN.
// ===========================================================================

#define NODE_NAME       "Zumo 2040"
#define BOARD_IS_ZUMO   1

// IDs de 11 bits (los de la practica): el Zumo manda 0x500, la Pico 2 W 0x200
#define CAN_TX_ID       0x500
#define CAN_PEER_ID     0x200   // = ID con el que el gateway manda la pose de la camara

// Pines (tabla de la Parte 1 del PDF)
#define MCP2515_SPI_PORT   spi0
#define MCP2515_PIN_MISO   0    // SO del MCP2515
#define MCP2515_PIN_CS     1
#define MCP2515_PIN_SCK    2
#define MCP2515_PIN_MOSI   3    // SI  (en el Zumo GP3 tambien es el dato de los LEDs RGB)
#define MCP2515_PIN_INT    28
#define MCP2515_SPI_BAUD   (1000 * 1000)

// Reloj de los LEDs RGB del Zumo: se mantiene en bajo para que no interfieran (practica, Parte 3f)
#define ZUMO_RGB_CLK_PIN   6

// Cristal del modulo MCP2515 (8.000 o 16.000). Se cambia en vivo con la tecla 'x'.
#define MCP2515_CRYSTAL_MHZ 8

// 1 = arranca en NORMAL (Parte 4); 0 = LOOPBACK (Partes 2 y 3)
#define CAN_START_IN_NORMAL_MODE 1

#endif // CONFIG_H
