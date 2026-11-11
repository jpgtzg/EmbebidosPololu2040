#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
// Replace these with the network you want the Pico W to join.
static const char *WIFI_SSID = "embedded";
static const char *WIFI_PASSWORD = "12345678";
static const uint16_t TELEMETRY_PORT = 5000;
 
// Assumes each telemetry line fits in a single pbuf (true for the JSON
// lines this listens for); a larger/fragmented UDP payload would need
// pbuf_copy_partial() over the whole pbuf chain instead of one segment.
static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                       	const ip_addr_t *addr, u16_t port) {
	(void)arg;
	(void)pcb;
	(void)addr;
	(void)port;
	if (p != NULL) {
    	printf("%.*s\n", p->len, (char *)p->payload);
    	pbuf_free(p);
	}
}
int main(void) {
	stdio_init_all();
 
	if (cyw43_arch_init()) {
    	printf("Wi-Fi init failed\n");
    	return -1;
	}
	cyw43_arch_enable_sta_mode();
 
	while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD,
                                           	CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
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
    	cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
	}
}
