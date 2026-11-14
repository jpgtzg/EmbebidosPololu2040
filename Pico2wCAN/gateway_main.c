// gateway_main.c - Pico 2 W: camara (Wi-Fi/UDP JSON) -> CAN -> Zumo
//
// Recibe el JSON de la camara por UDP (puerto 5000), toma la pose del marcador del
// Zumo (TRACK_MARKER_ID) y la manda por CAN con el formato de can_pose.h.
// Se debe usar la misma red Wi-Fi que Local_positioning.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "config.h"
#include "mcp2515.h"
#include "can_pose.h"

static const char *WIFI_SSID = "embedded";
static const char *WIFI_PASSWORD = "12345678";
#define TELEMETRY_PORT   5000
#define TRACK_MARKER_ID  13      // id del marcador que va sobre el Zumo

// ---- pose mas reciente (escrita por el callback UDP, leida por el lazo principal) ----
static volatile bool    s_new_pose = false;
static can_pose_t       s_pose;
static volatile uint8_t s_seq = 0;
static uint32_t         s_udp_rx = 0, s_no_marker = 0;

static bool get_num(const char *obj, const char *key, float *out) {
    const char *k = strstr(obj, key);
    if (!k) return false;
    char *end;
    float v = strtof(k + strlen(key), &end);
    if (end == k + strlen(key)) return false;
    *out = v;
    return true;
}

// Busca el marcador `want_id` dentro de {"markers":[{"id":..,"x":..,"y":..,"angle":..},...]}
static bool parse_pose(char *json, int want_id, can_pose_t *out) {
    char *p = json;
    while ((p = strstr(p, "\"id\":")) != NULL) {
        int id = atoi(p + 5);
        char *end = strchr(p, '}');
        if (!end) return false;
        if (id == want_id) {
            *end = '\0';                       // limita la busqueda a este marcador
            float x, y, a;
            bool ok = get_num(p, "\"x\":", &x) && get_num(p, "\"y\":", &y) &&
                      get_num(p, "\"angle\":", &a);
            *end = '}';
            if (!ok) return false;
            out->x_cm = x * 100.0f;            // la camara manda metros
            out->y_cm = y * 100.0f;
            out->theta_deg = a;
            float age = 0;
            out->age_ms = get_num(end, "\"frame_age_ms\":", &age) ? (age > 255 ? 255 : (uint8_t)age) : 0;
            return true;
        }
        p = end;
    }
    return false;
}

static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (!p) return;
    static char buf[1536];
    u16_t n = pbuf_copy_partial(p, buf, sizeof(buf) - 1, 0);
    buf[n] = '\0';
    pbuf_free(p);
    s_udp_rx++;

    can_pose_t pose;
    if (parse_pose(buf, TRACK_MARKER_ID, &pose)) {
        pose.seq = ++s_seq;
        s_pose = pose;
        s_new_pose = true;
    } else {
        s_no_marker++;
    }
}

int main(void) {
    stdio_init_all();
    absolute_time_t t0 = make_timeout_time_ms(4000);
    while (!stdio_usb_connected() && !time_reached(t0)) sleep_ms(50);
    printf("\n=== Gateway camara -> CAN | marcador %d | CAN ID 0x%03X ===\n", TRACK_MARKER_ID, CAN_TX_ID);

    // ---- CAN primero (no depende del Wi-Fi) ----
    while (!mcp2515_init()) {
        printf("[ERROR] MCP2515 no responde por SPI, reintentando...\n");
        sleep_ms(2000);
    }
    while (!mcp2515_set_bitrate_500kbps(MCP2515_CRYSTAL_MHZ)) {
        printf("[ERROR] No se pudo configurar el bit timing\n");
        sleep_ms(2000);
    }
    if (!mcp2515_set_mode(MCP2515_MODE_NORMAL)) printf("[ERROR] No entro a modo NORMAL\n");
    else printf("[OK] CAN en modo NORMAL, 500 kbit/s, cristal %u MHz\n", MCP2515_CRYSTAL_MHZ);

    // ---- Wi-Fi + UDP ----
    if (cyw43_arch_init()) { printf("Wi-Fi init failed\n"); return -1; }
    cyw43_arch_enable_sta_mode();
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
        printf("Wi-Fi connect failed, retrying...\n");
    printf("Wi-Fi conectado\n");
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);   // sin esto el broadcast llega con segundos de retraso

    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
    udp_recv(pcb, on_udp_packet, NULL);

    uint32_t sent = 0, failed = 0;
    absolute_time_t next_report = make_timeout_time_ms(1000);

    while (true) {
        cyw43_arch_poll();

        if (s_new_pose) {
            can_pose_t pose = s_pose;          // copia local
            s_new_pose = false;
            can_frame_t f = { .id = CAN_TX_ID };
            can_pose_pack(&pose, &f);
            if (mcp2515_send_message(&f)) sent++; else failed++;
        }

        if (time_reached(next_report)) {
            printf("UDP=%u sin_marcador=%u | CAN ok=%u fallo=%u | ultima x=%.1f y=%.1f th=%.1f\n",
                   (unsigned)s_udp_rx, (unsigned)s_no_marker, (unsigned)sent, (unsigned)failed,
                   s_pose.x_cm, s_pose.y_cm, s_pose.theta_deg);
            next_report = make_timeout_time_ms(1000);
        }

        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
    }
}
