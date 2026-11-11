#include <stdio.h>
#include "pico/stdlib.h"
#include "mcp2515.h"

static bool run_loopback_self_test(void) {
    printf("Running MCP2515 loopback self-test...\n");
    if (!mcp2515_set_mode(MCP2515_MODE_LOOPBACK)) {
        printf("Could not enter LOOPBACK mode\n");
        return false;
    }

    can_frame_t tx_frame = {
        .id = 0x123,
        .dlc = 4,
        .data = {0xDE, 0xAD, 0xBE, 0xEF}
    };

    if (!mcp2515_send_message(&tx_frame)) {
        printf("Loopback TX failed\n");
        return false;
    }

    sleep_ms(50);

    can_frame_t rx_frame;
    if (!mcp2515_read_message(&rx_frame)) {
        printf("Loopback RX failed: no message received\n");
        return false;
    }

    bool ok = (rx_frame.id == tx_frame.id) && (rx_frame.dlc == tx_frame.dlc);
    for (int i = 0; i < rx_frame.dlc && ok; i++) {
        if (rx_frame.data[i] != tx_frame.data[i]) ok = false;
    }

    printf("Loopback result: ID=0x%03X DLC=%d Data=", (unsigned)rx_frame.id, rx_frame.dlc);
    for (int i = 0; i < rx_frame.dlc; i++) printf("%02X ", rx_frame.data[i]);
    printf("-> %s\n", ok ? "PASS" : "FAIL");

    return ok;
}

int main(void) {
    stdio_init_all();
    sleep_ms(3000); // give the USB serial connection time to enumerate

    printf("\n--- Pico 2 W + MCP2515 CAN demo ---\n");
    mcp2515_init();
    mcp2515_set_bitrate_500kbps_8mhz();

    if (!run_loopback_self_test()) {
        printf("Self-test FAILED. Check wiring/crystal frequency before trusting the bus.\n");
    } else {
        printf("Self-test PASSED. Switching to NORMAL mode.\n");
    }

    if (!mcp2515_set_mode(MCP2515_MODE_NORMAL)) {
        printf("Could not enter NORMAL mode\n");
    }

    uint32_t counter = 0;
    absolute_time_t next_send = make_timeout_time_ms(1000);

    while (true) {
        if (absolute_time_diff_us(get_absolute_time(), next_send) <= 0) {
            can_frame_t frame = {
                .id = 0x200,
                .dlc = 2,
                .data = { (uint8_t)(counter >> 8), (uint8_t)(counter & 0xFF) }
            };
            if (mcp2515_send_message(&frame)) {
                printf("TX  ID=0x%03X  counter=%u\n", (unsigned)frame.id, (unsigned)counter);
            } else {
                printf("TX failed\n");
            }
            counter++;
            next_send = make_timeout_time_ms(1000);
        }

        if (mcp2515_message_available()) {
            can_frame_t rx;
            if (mcp2515_read_message(&rx)) {
                printf("RX  ID=0x%03X  DLC=%d  Data=", (unsigned)rx.id, rx.dlc);
                for (int i = 0; i < rx.dlc; i++) printf("%02X ", rx.data[i]);
                printf("\n");
            }
        }

        sleep_ms(10);
    }
}
