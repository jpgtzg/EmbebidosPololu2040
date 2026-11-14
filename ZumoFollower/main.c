// ZumoFollower - Part C: el Zumo sigue la ruta de waypoints usando la pose de la camara (CAN).
//
//  camara -> Wi-Fi -> Pico 2 W (Pico2wGateway) -> CAN -> ESTE programa -> motores
//
// ARRANQUE AUTOMATICO (no necesita USB ni botones):
//   1. Enciende el Zumo con bateria y pon el Zumo QUIETO sobre un waypoint de la ruta
//      (P1 = vuelta completa), con el frente hacia donde va la ruta (a < 60 deg).
//   2. Cuando la camara lo ve quieto y bien orientado, cuenta START_DELAY_S segundos
//      (el LED amarillo parpadea) para que te alejes, y arranca solo.
//   3. Corre desde ese waypoint P1 -> P2 -> ... -> P14 -> de regreso a P1 y se detiene.
// SEGURIDAD: se detiene solo si pierde la camara > 0.4 s, si se sale > 25 cm de la ruta o
// al volver a P1. El boton A (si lo alcanzas) tambien lo para.
//
// Con las ruedas en el aire: compilar con -DDRY_RUN=ON (calcula y imprime, sin mover motores).
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "motors.h"
#include "pololu_zumo_2040_robot.h"   // incluye button.h y yellow_led()
#include "config.h"
#include "mcp2515.h"
#include "waypoints.h"
#include "path.h"

#ifndef DRY_RUN
#define DRY_RUN 0
#endif
#define POSE_TIMEOUT_MS     400
#define START_MAX_DIST_CM   15.0f   // maxima distancia al waypoint de salida
#define START_MAX_HEAD_DEG  60.0f   // maximo error de orientacion al arrancar
#define START_STILL_CM      1.5f    // cuanto puede moverse mientras cuenta
#define START_DELAY_S       5
#define MAX_XTRACK_CM       25.0f

extern bool camera_get_pose(pose_t *out);   // camera_can.c

typedef enum { ST_IDLE, ST_RUNNING, ST_DONE, ST_ABORT } state_t;

static void motors_out(int32_t l, int32_t r) {
#if DRY_RUN
    (void)l; (void)r;
#else
    motors_set_speeds(l, r);
#endif
}

int main(void) {
    stdio_init_all();

    // Reloj de los LEDs RGB en bajo antes de usar SPI (GP3 = MOSI = dato de los LEDs)
    gpio_init(ZUMO_RGB_CLK_PIN);
    gpio_put(ZUMO_RGB_CLK_PIN, 0);
    gpio_set_dir(ZUMO_RGB_CLK_PIN, GPIO_OUT);

    motors_init();
    motors_out(0, 0);

    // Sin cable USB no espera: con bateria arranca de inmediato
    absolute_time_t t0 = make_timeout_time_ms(1500);
    while (!stdio_usb_connected() && !time_reached(t0)) sleep_ms(50);

    printf("\n=== ZumoFollower | BASE_SPEED=%d KP=%.1f LOOKAHEAD=%.1f cm%s ===\n",
           BASE_SPEED, (double)KP_TURN, (double)LOOKAHEAD_CM, DRY_RUN ? " | DRY_RUN (sin motores)" : "");

    while (!mcp2515_init()) { printf("[ERROR] MCP2515 no responde, reintentando...\n"); sleep_ms(1000); }
    while (!mcp2515_set_bitrate_500kbps(MCP2515_CRYSTAL_MHZ)) { printf("[ERROR] bit timing\n"); sleep_ms(1000); }
    if (!mcp2515_set_mode(MCP2515_MODE_NORMAL)) printf("[ERROR] CAN no entro a NORMAL\n");

    static path_t path;
    path_build(&path);
    printf("Ruta completa: %d puntos. Inicio y final = P1 (%.1f, %.1f)\n", path.n,
           (double)WAYPOINTS[WP_FINAL_IDX].x_cm, (double)WAYPOINTS[WP_FINAL_IDX].y_cm);

    state_t st = ST_IDLE;
    pose_t pose = {0};
    bool have_pose = false;
    absolute_time_t last_pose_t = get_absolute_time();
    absolute_time_t next_print = make_timeout_time_ms(300);
    absolute_time_t run_start = get_absolute_time();
    bool a_prev = false;
    drive_cmd_t cmd = {0};

    // arranque automatico
    bool      arming = false;          // contando para arrancar
    absolute_time_t arm_t0 = get_absolute_time();
    pose_t    arm_ref = {0};
    int       start_wp = WP_START_IDX;
    const char *why = "esperando pose";

    while (true) {
        // ---- boton A: paro de emergencia ----
        bool a_now = button_a_is_pressed();
        bool a_edge = a_now && !a_prev;
        a_prev = a_now;

        // ---- pose nueva de la camara ----
        bool fresh = false;
        pose_t p;
        if (camera_get_pose(&p)) { pose = p; have_pose = true; fresh = true; last_pose_t = get_absolute_time(); }
        bool stale = !have_pose || absolute_time_diff_us(last_pose_t, get_absolute_time()) > POSE_TIMEOUT_MS * 1000;

        if (a_edge && st == ST_RUNNING) {
            st = ST_ABORT; motors_out(0, 0); yellow_led(false); printf("[PARO] boton A\n");
        }

        // ---- IDLE: decidir si arrancar solo ----
        if (st == ST_IDLE) {
            bool ok = false;
            if (stale) {
                why = "sin pose de la camara";
            } else {
                float d;
                int k = path_nearest_wp(&pose, &d);
                path_t tmp;
                path_build_from(&tmp, k);
                float he = path_start_heading_error(&tmp, &pose);
                if (d > START_MAX_DIST_CM)              why = "lejos de un waypoint";
                else if (fabsf(he) > START_MAX_HEAD_DEG) why = "mal orientado";
                else { ok = true; start_wp = k; why = "listo"; }
            }
            if (!ok) {
                arming = false; yellow_led(false);
            } else if (!arming) {
                arming = true; arm_t0 = get_absolute_time(); arm_ref = pose;
                printf("[ARMADO] salida en P%d; arranca en %d s (no lo muevas)\n", WAYPOINTS[start_wp].id, START_DELAY_S);
            } else {
                float moved = hypotf(pose.x_cm - arm_ref.x_cm, pose.y_cm - arm_ref.y_cm);
                int64_t el_ms = absolute_time_diff_us(arm_t0, get_absolute_time()) / 1000;
                yellow_led((el_ms / 250) % 2 == 0);                 // parpadea mientras cuenta
                if (moved > START_STILL_CM) {                       // lo movieron: reinicia la cuenta
                    arm_t0 = get_absolute_time(); arm_ref = pose;
                } else if (el_ms >= START_DELAY_S * 1000) {
                    path_build_from(&path, start_wp);
                    st = ST_RUNNING; run_start = get_absolute_time(); arming = false;
                    yellow_led(true);
                    printf("[INICIO] desde P%d pose=(%.1f, %.1f, %.1f) puntos=%d\n", WAYPOINTS[start_wp].id,
                           (double)pose.x_cm, (double)pose.y_cm, (double)pose.theta_deg, path.n);
                }
            }
        }

        // ---- RUNNING: seguir la ruta ----
        if (st == ST_RUNNING) {
            if (stale) {
                motors_out(0, 0);                       // sin camara = sin movimiento
            } else if (fresh) {
                path_follow(&path, &pose, &cmd);
                if (path.done) {
                    motors_out(0, 0); st = ST_DONE; yellow_led(false);
                    printf("[FIN] vuelta completa, de regreso en P1, en %.1f s\n",
                           (double)(absolute_time_diff_us(run_start, get_absolute_time()) / 1e6f));
                } else if (cmd.xtrack_cm > MAX_XTRACK_CM) {
                    motors_out(0, 0); st = ST_ABORT; yellow_led(false);
                    printf("[PARO] fuera de la ruta (%.1f cm)\n", (double)cmd.xtrack_cm);
                } else {
                    motors_out(cmd.left, cmd.right);
                }
            }
        }

        if (time_reached(next_print)) {
            next_print = make_timeout_time_ms(250);
            printf("%s pose=(%6.1f,%6.1f,%6.1f)%s | seg=%2d/%d err=%6.1f xt=%4.1f | L=%5d R=%5d%s%s\n",
                   st == ST_IDLE ? "IDLE " : st == ST_RUNNING ? "RUN  " : st == ST_DONE ? "DONE " : "ABORT",
                   (double)pose.x_cm, (double)pose.y_cm, (double)pose.theta_deg, stale ? " [SIN POSE]" : "",
                   path.seg, path.n - 1, (double)cmd.err_deg, (double)cmd.xtrack_cm,
                   (int)cmd.left, (int)cmd.right,
                   st == ST_IDLE ? " | " : "", st == ST_IDLE ? why : "");
        }

        sleep_ms(2);
    }
}
