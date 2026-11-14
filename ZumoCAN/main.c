// CAN bus con MCP2515: Zumo 2040 <-> Raspberry Pi Pico 2 W
// Este main.c es IDENTICO en los dos proyectos; lo que cambia esta en config.h.
//
// Flujo:
//   1. Verifica que el MCP2515 conteste por SPI (si no, dice que revisar y reintenta).
//   2. Configura 500 kbit/s segun el cristal del modulo.
//   3. Self-test en LOOPBACK (Partes 2e / 3h del PDF).
//   4. Cada segundo manda CAN_TX_ID con un contador y muestra todo lo que recibe.
//
// Teclas en el monitor serial: n = NORMAL, l = LOOPBACK, t = repetir self-test,
// x = cambiar cristal 8/16 MHz, d = diagnostico, h = ayuda.

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "config.h"
#include "mcp2515.h"

static uint8_t s_crystal_mhz = MCP2515_CRYSTAL_MHZ;
static uint8_t s_run_mode    = CAN_START_IN_NORMAL_MODE ? MCP2515_MODE_NORMAL : MCP2515_MODE_LOOPBACK;

static void print_frame(const char *tag, const can_frame_t *f) {
    printf("%s ID=0x%03X  DLC=%d  Data=", tag, (unsigned)f->id, f->dlc);
    for (int i = 0; i < f->dlc; i++) printf("%02X ", f->data[i]);
    if (f->dlc == 2) printf(" (counter=%u)", (unsigned)((f->data[0] << 8) | f->data[1]));
    printf("\n");
}

static void print_help(void) {
    printf("\n=== %s + MCP2515 CAN | TX ID=0x%03X | espera ID=0x%03X ===\n",
           NODE_NAME, CAN_TX_ID, CAN_PEER_ID);
    printf("Pines: MISO=GP%d CS=GP%d SCK=GP%d MOSI=GP%d INT=GP%d | SPI %u kHz | cristal %u MHz | 500 kbit/s\n",
           MCP2515_PIN_MISO, MCP2515_PIN_CS, MCP2515_PIN_SCK, MCP2515_PIN_MOSI, MCP2515_PIN_INT,
           (unsigned)(MCP2515_SPI_BAUD / 1000), s_crystal_mhz);
    printf("Teclas: n=NORMAL  l=LOOPBACK  t=self-test  x=cristal 8/16 MHz  d=diagnostico  h=ayuda\n\n");
}

static void wait_for_usb_terminal(uint32_t max_ms) {
    // Espera a que se abra el monitor serial para no perder los primeros mensajes
    absolute_time_t deadline = make_timeout_time_ms(max_ms);
    while (!stdio_usb_connected() && !time_reached(deadline)) sleep_ms(50);
    sleep_ms(200);
}

static void explain_spi_failure(void) {
    uint8_t canstat = mcp2515_read_register(MCP2515_CANSTAT);
    uint8_t canctrl = mcp2515_read_register(MCP2515_CANCTRL);
    printf("[ERROR] El MCP2515 no responde por SPI. CANSTAT=0x%02X (esperado 0x80)  CANCTRL=0x%02X (esperado 0x87)\n",
           canstat, canctrl);
    if (canstat == 0xFF && canctrl == 0xFF) {
        printf("  >> MISO siempre en 1: SO del modulo no llega a GP%d, CS no llega a GP%d, o el modulo no tiene VCC/GND.\n",
               MCP2515_PIN_MISO, MCP2515_PIN_CS);
    } else if (canstat == 0x00 && canctrl == 0x00) {
        printf("  >> MISO siempre en 0: SO en corto a GND, modulo sin alimentacion, o SCK (GP%d) no conectado.\n",
               MCP2515_PIN_SCK);
    } else {
        printf("  >> Respuesta incoherente: revisa SCK=GP%d y MOSI=GP%d (no cruzados), GND comun y cables flojos.\n",
               MCP2515_PIN_SCK, MCP2515_PIN_MOSI);
    }
    printf("  >> Reintentando en 2 s...\n");
}

static bool configure_controller(void) {
    if (!mcp2515_set_bitrate_500kbps(s_crystal_mhz)) {
        printf("[ERROR] No se pudo configurar el bit timing (cristal %u MHz)\n", s_crystal_mhz);
        return false;
    }
    printf("[OK] Bit timing: 500 kbit/s con cristal de %u MHz\n", s_crystal_mhz);
    return true;
}

static void enter_run_mode(uint8_t mode) {
    s_run_mode = mode;
    if (mcp2515_set_mode(mode)) {
        printf("[OK] Modo %s%s\n", mcp2515_mode_name(mode),
               mode == MCP2515_MODE_LOOPBACK ? " (cada trama enviada se recibe a si misma)"
                                             : " (hablando con el otro nodo por CANH/CANL)");
    } else {
        printf("[ERROR] El MCP2515 no entro a modo %s (esta en %s)\n",
               mcp2515_mode_name(mode), mcp2515_mode_name(mcp2515_get_mode()));
    }
}

static bool run_loopback_self_test(void) {
    printf("Running MCP2515 loopback self-test...\n");
    if (!mcp2515_set_mode(MCP2515_MODE_LOOPBACK)) {
        printf("Loopback self-test FAILED: no entro a modo LOOPBACK\n");
        return false;
    }

    can_frame_t junk;
    while (mcp2515_read_message(&junk)) { }   // descarta tramas viejas

    can_frame_t tx_frame = { .id = 0x123, .dlc = 4, .data = { 0xDE, 0xAD, 0xBE, 0xEF } };
    if (!mcp2515_send_message(&tx_frame)) {
        printf("Loopback TX failed (TXB0CTRL=0x%02X)\n", mcp2515_last_tx_flags());
        return false;
    }

    can_frame_t rx_frame;
    bool received = false;
    absolute_time_t deadline = make_timeout_time_ms(50);
    while (!(received = mcp2515_read_message(&rx_frame)) && !time_reached(deadline)) sleep_ms(1);
    if (!received) {
        printf("Loopback RX failed: no message received\n");
        return false;
    }

    bool ok = rx_frame.id == tx_frame.id && rx_frame.dlc == tx_frame.dlc;
    for (int i = 0; i < rx_frame.dlc && ok; i++) ok = rx_frame.data[i] == tx_frame.data[i];

    printf("Loopback result: ID=0x%03X DLC=%d Data=", (unsigned)rx_frame.id, rx_frame.dlc);
    for (int i = 0; i < rx_frame.dlc; i++) printf("%02X ", rx_frame.data[i]);
    printf("-> %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

static void print_no_ack_hint(void) {
    printf("  >> Nadie dio ACK a la trama. Revisa, en este orden:\n");
    printf("     1. El otro nodo esta conectado, corriendo este programa y en modo NORMAL (tecla n).\n");
    printf("     2. CANH con CANH y CANL con CANL (no cruzados).\n");
    printf("     3. Mismo bitrate: cada nodo con SU cristal (tecla x si el modulo dice 16.000).\n");
    printf("     4. El TJA1050 alimentado a 5 V (el puente que soldaron) y el MCP2515 a 3.3 V.\n");
    printf("     5. Jumper J1 (120 ohm) puesto en ambos modulos.\n");
}

static void handle_keyboard(void) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT) return;

    switch (c) {
        case 'n': case 'N': enter_run_mode(MCP2515_MODE_NORMAL);   break;
        case 'l': case 'L': enter_run_mode(MCP2515_MODE_LOOPBACK); break;
        case 't': case 'T':
            run_loopback_self_test();
            enter_run_mode(s_run_mode);
            break;
        case 'x': case 'X':
            s_crystal_mhz = (s_crystal_mhz == 8) ? 16 : 8;
            if (configure_controller()) enter_run_mode(s_run_mode);
            break;
        case 'd': case 'D': mcp2515_print_diagnostics(); break;
        case 'h': case 'H': case '?': print_help(); break;
        default: break;
    }
}

int main(void) {
    stdio_init_all();

#if BOARD_IS_ZUMO
    // Antes de tocar SPI: reloj de los LEDs RGB en bajo (GP3 = MOSI = dato de los LEDs)
    gpio_init(ZUMO_RGB_CLK_PIN);
    gpio_put(ZUMO_RGB_CLK_PIN, 0);
    gpio_set_dir(ZUMO_RGB_CLK_PIN, GPIO_OUT);
#endif

    wait_for_usb_terminal(5000);
    print_help();

    while (!mcp2515_init()) {
        explain_spi_failure();
        sleep_ms(2000);
    }
    printf("[OK] MCP2515 responde por SPI\n");

    while (!configure_controller()) sleep_ms(2000);

    if (run_loopback_self_test()) {
        printf("Self-test PASSED.\n");
    } else {
        printf("Self-test FAILED. Presiona d para ver registros.\n");
    }
    enter_run_mode(s_run_mode);

    uint32_t counter = 0;
    uint32_t tx_fail_streak = 0;
    absolute_time_t next_send = make_timeout_time_ms(1000);

    while (true) {
        handle_keyboard();

        if (time_reached(next_send)) {
            can_frame_t frame = {
                .id = CAN_TX_ID,
                .dlc = 2,
                .data = { (uint8_t)(counter >> 8), (uint8_t)(counter & 0xFF) }
            };
            if (mcp2515_send_message(&frame)) {
                printf("TX  ID=0x%03X  counter=%u\n", (unsigned)frame.id, (unsigned)counter);
                tx_fail_streak = 0;
            } else {
                tx_fail_streak++;
                printf("TX  ID=0x%03X  FAILED (sin ACK)  TEC=%u REC=%u\n", (unsigned)frame.id,
                       mcp2515_read_register(MCP2515_TEC), mcp2515_read_register(MCP2515_REC));
                if (tx_fail_streak == 1 || tx_fail_streak % 10 == 0) print_no_ack_hint();
            }
            counter++;
            next_send = make_timeout_time_ms(1000);
        }

        can_frame_t rx;
        while (mcp2515_read_message(&rx)) print_frame("RX ", &rx);

        sleep_ms(1);
    }
}
