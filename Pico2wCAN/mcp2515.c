#include <stdio.h>
#include <string.h>
#include "mcp2515.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Tiempo maximo para que otro nodo de ACK a una trama. A 500 kbit/s una
// trama tarda < 0.3 ms, asi que 50 ms ya significa "nadie esta contestando".
#define MCP2515_TX_TIMEOUT_MS 50

static uint8_t s_last_tx_flags = 0;

static inline void cs_select(void)   { gpio_put(MCP2515_PIN_CS, 0); }
static inline void cs_deselect(void) { gpio_put(MCP2515_PIN_CS, 1); }

bool mcp2515_init(void) {
    // CS en alto ANTES de activar SPI para no mandarle basura al chip
    gpio_init(MCP2515_PIN_CS);
    gpio_put(MCP2515_PIN_CS, 1);
    gpio_set_dir(MCP2515_PIN_CS, GPIO_OUT);

    spi_init(MCP2515_SPI_PORT, MCP2515_SPI_BAUD);
    spi_set_format(MCP2515_SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);   // SPI modo 0,0
    gpio_set_function(MCP2515_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_MOSI, GPIO_FUNC_SPI);
    // SO del MCP2515 queda en alta impedancia con CS en alto; el pull-up evita
    // que MISO quede flotando (y la errata E9 del RP2350 con pull-down).
    gpio_pull_up(MCP2515_PIN_MISO);

    // INT no es necesario (se consulta todo por SPI); solo se reporta en diagnostico
    gpio_init(MCP2515_PIN_INT);
    gpio_set_dir(MCP2515_PIN_INT, GPIO_IN);
    gpio_pull_up(MCP2515_PIN_INT);

    mcp2515_reset();
    return mcp2515_probe();
}

void mcp2515_reset(void) {
    uint8_t cmd = MCP2515_CMD_RESET;
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    cs_deselect();
    sleep_ms(10);   // el oscilador necesita arrancar (OST = 128 ciclos) antes de hablarle
}

// Despues de un RESET el chip DEBE estar en modo CONFIG con CANCTRL = 0x87.
// Ademas se escribe/lee CNF1 para confirmar que MOSI y MISO funcionan.
bool mcp2515_probe(void) {
    if ((mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK) != MCP2515_MODE_CONFIG) return false;
    if (mcp2515_read_register(MCP2515_CANCTRL) != MCP2515_CANCTRL_RESET_VALUE) return false;

    static const uint8_t patterns[] = { 0x55, 0xAA, 0x00 };
    for (unsigned i = 0; i < sizeof patterns; i++) {
        mcp2515_write_register(MCP2515_CNF1, patterns[i]);
        if (mcp2515_read_register(MCP2515_CNF1) != patterns[i]) return false;
    }
    return true;
}

uint8_t mcp2515_read_register(uint8_t address) {
    uint8_t tx[3] = { MCP2515_CMD_READ, address, 0x00 };
    uint8_t rx[3];
    cs_select();
    spi_write_read_blocking(MCP2515_SPI_PORT, tx, rx, 3);
    cs_deselect();
    return rx[2];
}

void mcp2515_write_register(uint8_t address, uint8_t value) {
    uint8_t tx[3] = { MCP2515_CMD_WRITE, address, value };
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 3);
    cs_deselect();
}

void mcp2515_modify_register(uint8_t address, uint8_t mask, uint8_t value) {
    uint8_t tx[4] = { MCP2515_CMD_BIT_MODIFY, address, mask, value };
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, tx, 4);
    cs_deselect();
}

uint8_t mcp2515_read_status(void) {
    uint8_t tx[2] = { MCP2515_CMD_READ_STATUS, 0x00 };
    uint8_t rx[2];
    cs_select();
    spi_write_read_blocking(MCP2515_SPI_PORT, tx, rx, 2);
    cs_deselect();
    return rx[1];
}

bool mcp2515_set_mode(uint8_t mode) {
    mcp2515_modify_register(MCP2515_CANCTRL, MCP2515_MODE_MASK, mode);

    // A diferencia del codigo original, nunca se queda colgado: maximo 100 ms
    absolute_time_t deadline = make_timeout_time_ms(100);
    do {
        if (mcp2515_get_mode() == mode) return true;
        sleep_ms(1);
    } while (!time_reached(deadline));
    return false;
}

uint8_t mcp2515_get_mode(void) {
    return mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK;
}

const char *mcp2515_mode_name(uint8_t mode) {
    switch (mode & MCP2515_MODE_MASK) {
        case MCP2515_MODE_NORMAL:     return "NORMAL";
        case MCP2515_MODE_SLEEP:      return "SLEEP";
        case MCP2515_MODE_LOOPBACK:   return "LOOPBACK";
        case MCP2515_MODE_LISTENONLY: return "LISTEN-ONLY";
        case MCP2515_MODE_CONFIG:     return "CONFIG";
        default:                      return "?";
    }
}

// Bit timing para 500 kbit/s (bit = 2 us):
//  8 MHz: BRP=0 -> TQ = 2/8MHz  = 250 ns, 8 TQ  = Sync 1 + Prop 1 + PS1 3 + PS2 3
// 16 MHz: BRP=0 -> TQ = 2/16MHz = 125 ns, 16 TQ = Sync 1 + Prop 1 + PS1 7 + PS2 7
// LOS DOS NODOS deben quedar en 500 kbit/s: cada uno se configura con SU cristal.
bool mcp2515_set_bitrate_500kbps(uint8_t crystal_mhz) {
    uint8_t cnf1, cnf2, cnf3;
    switch (crystal_mhz) {
        case 8:  cnf1 = 0x00; cnf2 = 0x90; cnf3 = 0x02; break;
        case 16: cnf1 = 0x00; cnf2 = 0xF0; cnf3 = 0x86; break;
        default: return false;
    }

    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG)) return false;   // CNFx solo se escriben en CONFIG

    mcp2515_write_register(MCP2515_CNF1, cnf1);
    mcp2515_write_register(MCP2515_CNF2, cnf2);
    mcp2515_write_register(MCP2515_CNF3, cnf3);

    // RXB0: acepta cualquier trama (RXM=11) y si esta lleno pasa a RXB1 (BUKT=1)
    mcp2515_write_register(MCP2515_RXB0CTRL, 0x64);
    mcp2515_write_register(MCP2515_RXB1CTRL, 0x60);
    mcp2515_write_register(MCP2515_CANINTE, MCP2515_CANINTF_RX0IF | MCP2515_CANINTF_RX1IF);
    mcp2515_write_register(MCP2515_CANINTF, 0x00);
    mcp2515_write_register(MCP2515_EFLG, 0x00);

    return mcp2515_read_register(MCP2515_CNF1) == cnf1 &&
           mcp2515_read_register(MCP2515_CNF2) == cnf2 &&
           mcp2515_read_register(MCP2515_CNF3) == cnf3;
}

// Cancela la trama de TXB0 (p. ej. cuando nadie le da ACK y el MCP2515 la
// reintentaria para siempre).
static void abort_tx0(void) {
    mcp2515_modify_register(MCP2515_TXB0CTRL, MCP2515_TXB_TXREQ, 0x00);
    absolute_time_t deadline = make_timeout_time_ms(10);
    while ((mcp2515_read_register(MCP2515_TXB0CTRL) & MCP2515_TXB_TXREQ) && !time_reached(deadline)) {
        sleep_us(100);
    }
    mcp2515_modify_register(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0x00);
}

bool mcp2515_send_message(const can_frame_t *frame) {
    if (frame->dlc > 8) return false;

    // Si quedo una trama pendiente de antes, el buffer no se puede reescribir
    if (mcp2515_read_register(MCP2515_TXB0CTRL) & MCP2515_TXB_TXREQ) abort_tx0();

    // LOAD TX BUFFER: SIDH, SIDL, EID8, EID0, DLC, D0..D7 en una sola rafaga SPI
    uint8_t buf[1 + 5 + 8];
    buf[0] = MCP2515_CMD_LOAD_TXB0;
    if (frame->extended) {
        uint32_t id = frame->id & 0x1FFFFFFFu;
        buf[1] = (uint8_t)(id >> 21);                                         // SID10..3
        buf[2] = (uint8_t)((((id >> 18) & 0x07) << 5) | 0x08 | ((id >> 16) & 0x03)); // SID2..0, EXIDE, EID17..16
        buf[3] = (uint8_t)(id >> 8);                                          // EID15..8
        buf[4] = (uint8_t)id;                                                 // EID7..0
    } else {
        uint32_t id = frame->id & 0x7FFu;
        buf[1] = (uint8_t)(id >> 3);            // SID10..3
        buf[2] = (uint8_t)((id & 0x07) << 5);   // SID2..0
        buf[3] = 0;
        buf[4] = 0;
    }
    buf[5] = frame->dlc;
    memcpy(&buf[6], frame->data, frame->dlc);

    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, buf, 6u + frame->dlc);
    cs_deselect();

    mcp2515_modify_register(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0x00);

    uint8_t cmd = MCP2515_CMD_RTS_TXB0;
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    cs_deselect();

    // TXREQ se limpia solo cuando la trama salio Y alguien dio ACK.
    // (El codigo original tenia un bug: su contador hacia underflow y
    // reportaba exito aunque la trama nunca saliera.)
    absolute_time_t deadline = make_timeout_time_ms(MCP2515_TX_TIMEOUT_MS);
    while (mcp2515_read_register(MCP2515_TXB0CTRL) & MCP2515_TXB_TXREQ) {
        if (time_reached(deadline)) {
            s_last_tx_flags = mcp2515_read_register(MCP2515_TXB0CTRL);
            abort_tx0();
            return false;
        }
        sleep_us(100);
    }

    mcp2515_modify_register(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0x00);
    s_last_tx_flags = 0;
    return true;
}

uint8_t mcp2515_last_tx_flags(void) {
    return s_last_tx_flags;
}

bool mcp2515_message_available(void) {
    // READ STATUS: bit0 = RX0IF, bit1 = RX1IF
    return (mcp2515_read_status() & 0x03) != 0;
}

bool mcp2515_read_message(can_frame_t *frame) {
    uint8_t status = mcp2515_read_status();
    uint8_t cmd;
    if (status & 0x01)      cmd = MCP2515_CMD_READ_RXB0;
    else if (status & 0x02) cmd = MCP2515_CMD_READ_RXB1;
    else                    return false;

    // READ RX BUFFER: SIDH, SIDL, EID8, EID0, DLC, D0..D7. El flag RXnIF se
    // limpia solo al subir CS, asi no se pierde ninguna trama.
    uint8_t tx[14] = { cmd };
    uint8_t rx[14];
    cs_select();
    spi_write_read_blocking(MCP2515_SPI_PORT, tx, rx, sizeof rx);
    cs_deselect();

    uint8_t sidh = rx[1], sidl = rx[2];
    frame->extended = (sidl & 0x08) != 0;   // bit IDE
    if (frame->extended) {
        frame->id = ((uint32_t)sidh << 21) | ((uint32_t)(sidl >> 5) << 18) |
                    ((uint32_t)(sidl & 0x03) << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    } else {
        frame->id = ((uint32_t)sidh << 3) | (sidl >> 5);
    }

    frame->dlc = rx[5] & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    memcpy(frame->data, &rx[6], frame->dlc);
    return true;
}

void mcp2515_print_diagnostics(void) {
    uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
    uint8_t eflg    = mcp2515_read_register(MCP2515_EFLG);
    uint8_t tec     = mcp2515_read_register(MCP2515_TEC);
    uint8_t rec     = mcp2515_read_register(MCP2515_REC);

    printf("---- MCP2515 diagnostico ----\n");
    printf("CANSTAT=0x%02X (modo %s)  CANCTRL=0x%02X\n",
           canstat, mcp2515_mode_name(canstat), mcp2515_read_register(MCP2515_CANCTRL));
    printf("CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X\n",
           mcp2515_read_register(MCP2515_CNF1), mcp2515_read_register(MCP2515_CNF2),
           mcp2515_read_register(MCP2515_CNF3));
    printf("CANINTE=0x%02X CANINTF=0x%02X TXB0CTRL=0x%02X (ultimo fallo TX: 0x%02X)\n",
           mcp2515_read_register(MCP2515_CANINTE), mcp2515_read_register(MCP2515_CANINTF),
           mcp2515_read_register(MCP2515_TXB0CTRL), s_last_tx_flags);
    printf("TEC=%u REC=%u EFLG=0x%02X%s%s%s%s%s\n", tec, rec, eflg,
           (eflg & MCP2515_EFLG_TXBO)   ? " BUS-OFF" : "",
           (eflg & MCP2515_EFLG_TXEP)   ? " TX-ERROR-PASSIVE" : "",
           (eflg & MCP2515_EFLG_RXEP)   ? " RX-ERROR-PASSIVE" : "",
           (eflg & MCP2515_EFLG_RX0OVR) ? " RX0-OVERFLOW" : "",
           (eflg & MCP2515_EFLG_RX1OVR) ? " RX1-OVERFLOW" : "");
    printf("Pin INT (GP%d) = %d  (0 = hay trama esperando)\n",
           MCP2515_PIN_INT, gpio_get(MCP2515_PIN_INT));
    printf("-----------------------------\n");

    // Los bits de overflow se quedan pegados hasta limpiarlos a mano
    if (eflg & (MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR)) {
        mcp2515_modify_register(MCP2515_EFLG, MCP2515_EFLG_RX0OVR | MCP2515_EFLG_RX1OVR, 0x00);
    }
}
