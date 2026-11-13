#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "vision_parcer.h"
#include <stdio.h>

// Replace these with the network you want the Pico W to join.
static const char *WIFI_SSID = "embedded";
static const char *WIFI_PASSWORD = "12345678";
static const uint16_t TELEMETRY_PORT = 5000;

#define MAX_TOKENS 128

static vision_packet_t latest_packet;
static volatile bool packet_ready = false;

static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
  (void)arg;
  (void)pcb;
  (void)addr;
  (void)port;
  if (p == NULL)
    return;

  static char json[512];
  u16_t len = pbuf_copy_partial(p, json, sizeof(json) - 1, 0);
  json[len] = '\0';
  pbuf_free(p);

  static jsmntok_t tokens[MAX_TOKENS];
  jsmn_parser parser;
  jsmn_init(&parser);
  int r = jsmn_parse(&parser, json, len, tokens, MAX_TOKENS);
  if (r < 1) {
    printf("JSON parse failed: %d\n", r);
    return;
  }

  vision_packet_t packet = {0};
  if (!parse_packet(json, tokens, &packet)) {
    printf("Not a valid packet\n");
    return;
  }

  latest_packet = packet;
  packet_ready = true;
}

int main(void) {
  stdio_init_all();

  if (cyw43_arch_init()) {
    printf("Wi-Fi init failed\n");
    return -1;
  }
  cyw43_arch_enable_sta_mode();

  while (cyw43_arch_wifi_connect_timeout_ms(
             WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
    printf("Wi-Fi connect failed, retrying...\n");
  }
  printf("Connected\n");

  // Equivalent of Arduino's WiFi.noLowPowerMode(): without this, the
  // radio buffers broadcast packets until periodic wake-ups instead of
  // receiving them promptly, causing multi-second delays.
  cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
  struct udp_pcb *pcb = udp_new();
  udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
  udp_recv(pcb, on_udp_packet, NULL);
  while (true) {
    cyw43_arch_poll();
    if (packet_ready) {
      packet_ready = false;
      printf("t=%.3f detected=%d markers=%d\n", latest_packet.timestamp,
             latest_packet.board_detected, latest_packet.marker_count);
      for (int i = 0; i < latest_packet.marker_count; i++) {
        marker_t *m = &latest_packet.markers[i];
        printf("  id=%d x=%.3f y=%.3f angle=%.1f\n", m->id, m->x, m->y,
               m->angle);
      }
    }
    cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
  }
}
