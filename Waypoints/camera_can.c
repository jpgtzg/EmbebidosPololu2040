// Lado del Zumo: camera_get_pose() lee por CAN la pose que manda el gateway (Pico 2 W).
#include "waypoints.h"
#include "config.h"
#include "mcp2515.h"
#include "can_pose.h"

// Devuelve true si llego una pose nueva; si hay varias acumuladas se queda con la ultima.
bool camera_get_pose(pose_t *out) {
    static uint8_t last_seq = 0;
    static bool    have_seq = false;
    bool got = false;
    can_frame_t f;
    can_pose_t  p;

    while (mcp2515_read_message(&f)) {
        if (f.extended || f.id != CAN_PEER_ID) continue;
        if (!can_pose_unpack(&f, &p)) continue;
        if (have_seq && p.seq == last_seq) continue;
        last_seq = p.seq; have_seq = true;
        out->x_cm = p.x_cm; out->y_cm = p.y_cm; out->theta_deg = p.theta_deg;
        got = true;
    }
    return got;
}
