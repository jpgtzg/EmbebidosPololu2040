// camera_can.c - lado del ZUMO: implementa el hook camera_get_pose() leyendo la pose
// que manda el gateway (Pico 2 W) por CAN. Requiere el mcp2515.c del proyecto del Zumo
// y su config.h (CAN_PEER_ID = 0x200, el ID que transmite el gateway).
#include "waypoints.h"
#include "config.h"
#include "mcp2515.h"
#include "can_pose.h"

// Devuelve true si llego una pose NUEVA desde la ultima llamada.
// Si llegan varias tramas acumuladas se queda con la ultima (la mas fresca).
bool camera_get_pose(pose_t *out) {
    static uint8_t last_seq = 0;
    static bool    have_seq = false;
    bool got = false;
    can_frame_t f;
    can_pose_t  p;

    while (mcp2515_read_message(&f)) {
        if (f.extended || f.id != CAN_PEER_ID) continue;
        if (!can_pose_unpack(&f, &p)) continue;
        if (have_seq && p.seq == last_seq) continue;     // repetida
        last_seq = p.seq; have_seq = true;
        out->x_cm = p.x_cm; out->y_cm = p.y_cm; out->theta_deg = p.theta_deg;
        got = true;
    }
    return got;
}
