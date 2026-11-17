#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "mcp2515.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "vision_parcer.h"
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/_pthreadtypes.h>

#define MAX_TOKENS 128
#define ROBOT_MARKER 13

// CAN frame layout (little-endian, standard 11-bit IDs, 500 kbps):
//   CAN_ID_POSE: [0]=seq [1]=flags [2..3]=x mm [4..5]=y mm [6..7]=angle 0.1 deg
//   CAN_ID_TIME: [0]=seq [1..4]=epoch ms (low 32 bits) [5..7]=unused
#define CAN_ID_POSE 0x300
#define CAN_ID_TIME 0x301
#define FLAG_BOARD_DETECTED 0x01
#define FLAG_MARKER_FOUND 0x02

// Replace these with the network you want the Pico W to join.
static const char *WIFI_SSID = "embedded";
static const char *WIFI_PASSWORD = "12345678";
static const uint16_t TELEMETRY_PORT = 5000;

static vision_packet_t latest_packet;
static bool packet_ready = false;
static jsmn_parser parser;

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

static uint8_t can_seq = 0;

static int16_t to_i16(float v) {
  long r = lroundf(v);
  if (r > INT16_MAX)
    r = INT16_MAX;
  if (r < INT16_MIN)
    r = INT16_MIN;
  return (int16_t)r;
}

static void put_i16(uint8_t *dst, int16_t v) {
  dst[0] = (uint8_t)((uint16_t)v & 0xFF);
  dst[1] = (uint8_t)((uint16_t)v >> 8);
}

static const marker_t *find_robot_marker(const vision_packet_t *p) {
  for (int i = 0; i < p->marker_count; i++) {
    if (p->markers[i].id == ROBOT_MARKER)
      return &p->markers[i];
  }
  return NULL;
}

static void send_packet_over_can(const vision_packet_t *p) {
  const marker_t *m = find_robot_marker(p);
  uint8_t seq = can_seq++;

  uint8_t flags = 0;
  if (p->board_detected)
    flags |= FLAG_BOARD_DETECTED;
  if (m)
    flags |= FLAG_MARKER_FOUND;

  can_frame_t pose = {.id = CAN_ID_POSE, .dlc = 8};
  pose.data[0] = seq;
  pose.data[1] = flags;
  put_i16(&pose.data[2], m ? to_i16(m->x * 1000.0f) : 0);
  put_i16(&pose.data[4], m ? to_i16(m->y * 1000.0f) : 0);
  put_i16(&pose.data[6], m ? to_i16(m->angle * 10.0f) : 0);

  if (!mcp2515_send_message(&pose)) {
    printf("CAN pose TX failed\n");
    return;
  }

  uint32_t ms = (uint32_t)((uint64_t)(p->timestamp * 1000.0 + 0.5));
  can_frame_t time = {.id = CAN_ID_TIME, .dlc = 5};
  time.data[0] = seq;
  time.data[1] = (uint8_t)(ms & 0xFF);
  time.data[2] = (uint8_t)((ms >> 8) & 0xFF);
  time.data[3] = (uint8_t)((ms >> 16) & 0xFF);
  time.data[4] = (uint8_t)((ms >> 24) & 0xFF);

  if (!mcp2515_send_message(&time))
    printf("CAN time TX failed\n");
}

static void process_packet() {
  printf("t=%.3f detected=%d markers=%d\n", latest_packet.timestamp,
         latest_packet.board_detected, latest_packet.marker_count);
  send_packet_over_can(&latest_packet);
}

int main(void) {
  stdio_init_all();

  mcp2515_init();
  if (!mcp2515_set_bitrate_500kbps_8mhz())
    printf("CAN bitrate setup failed\n");
  if (!mcp2515_set_mode(MCP2515_MODE_NORMAL))
    printf("CAN could not enter NORMAL mode\n");

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
      process_packet();
    }
    cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
  }
}
