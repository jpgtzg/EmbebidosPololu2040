#include "mcp2515.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// The Zumo's RGB LEDs share GP3 (MOSI) as their data line. Keep their clock
// (GP6) low so the LEDs ignore the SPI traffic meant for the MCP2515.
#define ZUMO_RGB_CLOCK_PIN 6

static inline void cs_select(void)   { gpio_put(MCP2515_PIN_CS, 0); }
static inline void cs_deselect(void) { gpio_put(MCP2515_PIN_CS, 1); }

void mcp2515_init(void) {
    /* Keep the Zumo RGB LED clock inactive. */
    gpio_init(ZUMO_RGB_CLOCK_PIN);
    gpio_put(ZUMO_RGB_CLOCK_PIN, 0);
    gpio_set_dir(ZUMO_RGB_CLOCK_PIN, GPIO_OUT);

    /* Establish CS high before starting SPI. */
    gpio_init(MCP2515_PIN_CS);
    gpio_put(MCP2515_PIN_CS, 1);
    gpio_set_dir(MCP2515_PIN_CS, GPIO_OUT);

    spi_init(MCP2515_SPI_PORT, MCP2515_SPI_BAUD);
    spi_set_format(MCP2515_SPI_PORT, 8,
                   SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP2515_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MCP2515_PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(MCP2515_PIN_INT);
    gpio_set_dir(MCP2515_PIN_INT, GPIO_IN);
    gpio_pull_up(MCP2515_PIN_INT);

    mcp2515_reset();
}

void mcp2515_reset(void) {
    uint8_t cmd = MCP2515_CMD_RESET;
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    cs_deselect();
    sleep_ms(10);
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

// Returns false if the controller did not enter the requested mode within
// 100 ms (the PDF version looped forever when the chip did not answer).
bool mcp2515_set_mode(uint8_t mode) {
    mcp2515_modify_register(MCP2515_CANCTRL, MCP2515_MODE_MASK, mode);

    absolute_time_t deadline = make_timeout_time_ms(100);
    do {
        if ((mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK) == mode) {
            return true;
        }
        sleep_ms(1);
    } while (!time_reached(deadline));
    return false;
}

// CNF values for 500 kbps with an 8 MHz crystal (the crystal fitted to
// most low-cost MCP2515 breakout boards). If your module uses a 16 MHz crystal
// instead, use CNF1=0x00, CNF2=0xF0, CNF3=0x86. Both nodes must match.
bool mcp2515_set_bitrate_500kbps_8mhz(void) {
    if (!mcp2515_set_mode(MCP2515_MODE_CONFIG)) {   // CNF registers are only writable in config mode
        return false;
    }
    mcp2515_write_register(MCP2515_CNF1, 0x00);
    mcp2515_write_register(MCP2515_CNF2, 0x90);
    mcp2515_write_register(MCP2515_CNF3, 0x02);
    mcp2515_write_register(MCP2515_CANINTE, MCP2515_CANINTF_RX0IF);
    mcp2515_write_register(MCP2515_RXB0CTRL, 0x60); // accept all messages, no filters/masks
    return true;
}

bool mcp2515_send_message(const can_frame_t *frame) {
    if (frame->dlc > 8) return false;

    uint8_t sidh = (uint8_t)(frame->id >> 3);
    uint8_t sidl = (uint8_t)((frame->id & 0x07) << 5);

    mcp2515_write_register(MCP2515_TXB0SIDH, sidh);
    mcp2515_write_register(MCP2515_TXB0SIDL, sidl);
    mcp2515_write_register(MCP2515_TXB0DLC, frame->dlc);

    for (uint8_t i = 0; i < frame->dlc; i++) {
        mcp2515_write_register(MCP2515_TXB0D0 + i, frame->data[i]);
    }

    uint8_t cmd = MCP2515_CMD_RTS_TXB0;   // request-to-send TXB0
    cs_select();
    spi_write_blocking(MCP2515_SPI_PORT, &cmd, 1);
    cs_deselect();

    // TXREQ clears once the frame went out AND some node acknowledged it.
    // (The PDF's `while (... && timeout--)` underflowed, so it reported success
    // even when nothing was sent.) Give up after ~50 ms: with no peer the
    // controller would retry the frame forever.
    bool sent = false;
    for (uint32_t waited_us = 0; waited_us < 50000; waited_us += 100) {
        if (!(mcp2515_read_register(MCP2515_TXB0CTRL) & MCP2515_TXB_TXREQ)) {
            sent = true;
            break;
        }
        sleep_us(100);
    }

    if (!sent) {
        // Abort the pending frame so it does not block the next transmission
        mcp2515_modify_register(MCP2515_TXB0CTRL, MCP2515_TXB_TXREQ, 0x00);
    }
    mcp2515_modify_register(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0x00);
    return sent;
}

bool mcp2515_message_available(void) {
    return (mcp2515_read_register(MCP2515_CANINTF) & MCP2515_CANINTF_RX0IF) != 0;
}

bool mcp2515_read_message(can_frame_t *frame) {
    if (!mcp2515_message_available()) return false;

    uint8_t sidh = mcp2515_read_register(MCP2515_RXB0SIDH);
    uint8_t sidl = mcp2515_read_register(MCP2515_RXB0SIDL);
    frame->id = ((uint32_t)sidh << 3) | (sidl >> 5);

    frame->dlc = mcp2515_read_register(MCP2515_RXB0DLC) & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    for (uint8_t i = 0; i < frame->dlc; i++) {
        frame->data[i] = mcp2515_read_register(MCP2515_RXB0D0 + i);
    }

    mcp2515_modify_register(MCP2515_CANINTF, MCP2515_CANINTF_RX0IF, 0x00);
    return true;
}
