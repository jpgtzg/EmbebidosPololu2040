// ZumoFollower: sigue la ruta de waypoints con la pose de la camara recibida por CAN.
// Arranca solo cuando la camara lo ve cerca de un waypoint y bien orientado, y se detiene
// al volver a P1, al perder la camara o al salirse de la ruta. Boton A = paro de emergencia.
// LED amarillo sin USB: parpadeo lento = sin pose; rapido = hay pose pero no esta listo; fijo = corriendo.
// Opciones de CMake: -DDRY_RUN=ON calcula e imprime sin mover los motores;
// -DMODE_A=ON hace la prueba de la Parte A (avanzar A_DISTANCE_CM en +y desde donde este).
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "motors.h"
#include "pololu_zumo_2040_robot.h"
#include "config.h"
#include "mcp2515.h"
#include "waypoints.h"
#include "path.h"

#ifndef DRY_RUN
#define DRY_RUN 0
#endif
#ifndef MODE_A
#define MODE_A 0
#endif
#ifndef A_DISTANCE_CM
#define A_DISTANCE_CM       50.0f
#endif
#define POSE_TIMEOUT_MS     400
#define START_MAX_DIST_CM   15.0f
#define START_MAX_HEAD_DEG  60.0f
#define MAX_XTRACK_CM       25.0f

extern bool camera_get_pose(pose_t *out);

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

    // GP3 (MOSI) tambien es el dato de los LEDs RGB: su reloj se mantiene en bajo.
    gpio_init(ZUMO_RGB_CLK_PIN);
    gpio_put(ZUMO_RGB_CLK_PIN, 0);
    gpio_set_dir(ZUMO_RGB_CLK_PIN, GPIO_OUT);

    motors_init();
    motors_out(0, 0);

    absolute_time_t t0 = make_timeout_time_ms(300);
    while (!stdio_usb_connected() && !time_reached(t0)) sleep_ms(50);

    printf("\n=== ZumoFollower | recta=%d curva=%d KP=%.1f FF=%.2f LOOKAHEAD=%.1f cm%s%s ===\n",
           STRAIGHT_SPEED, CURVE_SPEED, (double)KP_TURN, (double)FF_GAIN, (double)LOOKAHEAD_CM,
           DRY_RUN ? " | DRY_RUN" : "", MODE_A ? " | MODO A" : "");

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
    const char *why = "esperando pose";

    while (true) {
        bool a_now = button_a_is_pressed();
        bool a_edge = a_now && !a_prev;
        a_prev = a_now;

        bool fresh = false;
        if (camera_get_pose(&pose)) { have_pose = true; fresh = true; last_pose_t = get_absolute_time(); }
        bool stale = !have_pose || absolute_time_diff_us(last_pose_t, get_absolute_time()) > POSE_TIMEOUT_MS * 1000;

        if (a_edge && st == ST_RUNNING) {
            st = ST_ABORT; motors_out(0, 0); yellow_led(false); printf("[PARO] boton A\n");
        }

        if (st == ST_IDLE) {
            bool ok = false;
            if (stale) {
                why = "sin pose de la camara";
            } else {
#if MODE_A
                path_build_line(&path, pose.x_cm, pose.y_cm, pose.x_cm, pose.y_cm + A_DISTANCE_CM);
                float he = path_start_heading_error(&path, &pose);
                if (fabsf(he) > START_MAX_HEAD_DEG) why = "mal orientado (debe mirar a +y)";
                else ok = true;
#else
                float d;
                int k = path_nearest_wp(&pose, &d);
                path_build_from(&path, k);
                float he = path_start_heading_error(&path, &pose);
                if (d > START_MAX_DIST_CM)               why = "lejos de un waypoint";
                else if (fabsf(he) > START_MAX_HEAD_DEG) why = "mal orientado";
                else ok = true;
#endif
            }
            if (ok) {
                st = ST_RUNNING; run_start = get_absolute_time();
                yellow_led(true);
                printf("[INICIO] pose=(%.1f, %.1f, %.1f) puntos=%d\n",
                       (double)pose.x_cm, (double)pose.y_cm, (double)pose.theta_deg, path.n);
            } else {
                uint32_t ms = to_ms_since_boot(get_absolute_time());
                yellow_led(stale ? (ms / 500) % 2 == 0 : (ms / 100) % 2 == 0);
            }
        }

        if (st == ST_RUNNING) {
            if (stale) {
                motors_out(0, 0);
            } else if (fresh) {
                path_follow(&path, &pose, &cmd);
                float t_s = absolute_time_diff_us(run_start, get_absolute_time()) / 1e6f;
                for (int k = 0; k < WP_COUNT; k++) {
                    if (cmd.wp_new_reached & (1u << k))
                        printf("[P%d] alcanzado a los %.1f s (a %.1f cm)\n", WAYPOINTS[k].id, (double)t_s, (double)path.wp_dist[k]);
                    if (cmd.wp_new_missed & (1u << k))
                        printf("[P%d] OMITIDO\n", WAYPOINTS[k].id);
                }
                if (path.done) {
                    motors_out(0, 0); st = ST_DONE; yellow_led(false);
                    printf("[FIN] ruta completa en %.1f s | pose final=(%.1f, %.1f, %.1f)\n",
                           (double)t_s, (double)pose.x_cm, (double)pose.y_cm, (double)pose.theta_deg);
                    printf("Waypoints alcanzados: %d de %d\n", path_wp_reached(&path), path.wp_n);
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
            printf("%s pose=(%6.1f,%6.1f,%6.1f)%s | seg=%2d/%d sig=P%d err=%6.1f xt=%4.1f | v=%4d L=%5d R=%5d%s%s\n",
                   st == ST_IDLE ? "IDLE " : st == ST_RUNNING ? "RUN  " : st == ST_DONE ? "DONE " : "ABORT",
                   (double)pose.x_cm, (double)pose.y_cm, (double)pose.theta_deg, stale ? " [SIN POSE]" : "",
                   path.seg, path.n - 1, st == ST_RUNNING ? path_next_wp_id(&path) : 0,
                   (double)cmd.err_deg, (double)cmd.xtrack_cm,
                   (int)cmd.speed, (int)cmd.left, (int)cmd.right,
                   st == ST_IDLE ? " | " : "", st == ST_IDLE ? why : "");
        }

        sleep_ms(2);
    }
}
