// path.c - Part C: construccion de la ruta y control de seguimiento (pure pursuit)
#include <math.h>
#include "path.h"

#define D2R 0.01745329252f
#define R2D 57.2957795f

static float wrap180(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static void add(path_t *P, float x, float y) {
    if (P->n < PATH_MAX_POINTS) { P->p[P->n].x = x; P->p[P->n].y = y; P->n++; }
}

// Agrega el interior de un arco: centro (cx,cy), radio r, desde el angulo a0 con
// barrido total sweep (grados, con signo: + = antihorario).
static void add_arc(path_t *P, float cx, float cy, float r, float a0, float sweep) {
    int steps = (int)ceilf(fabsf(sweep) / ARC_STEP_DEG);
    for (int k = 1; k < steps; k++) {
        float a = (a0 + sweep * k / steps) * D2R;
        add(P, cx + r * cosf(a), cy + r * sinf(a));
    }
}

// Entrada E (con rumbo theta) y salida X: agrega E, el arco y X.
static void add_turn(path_t *P, const waypoint_t *E, const waypoint_t *X, bool is_u) {
    add(P, E->x_cm, E->y_cm);

    float hx = cosf(E->theta_deg * D2R), hy = sinf(E->theta_deg * D2R);
    float cx = X->x_cm - E->x_cm, cy = X->y_cm - E->y_cm;
    float chord2 = cx * cx + cy * cy;
    float chord = sqrtf(chord2);
    if (chord < 6.0f) {               // entrada y salida casi en el mismo punto: sin arco
        add(P, X->x_cm, X->y_cm);
        return;
    }
    // Circulo tangente al rumbo de entrada que pasa exactamente por E y por X.
    // Centro = E + s*n, con n = normal izquierda del rumbo; s > 0 gira a la izquierda (antihorario).
    // (Para una U con la cuerda perpendicular al rumbo da el semicirculo de diametro E-X.)
    float nx = -hy, ny = hx;
    float nc = nx * cx + ny * cy;     // componente de la cuerda sobre la normal
    if (fabsf(nc) < 0.02f * chord) {  // casi recta: sin arco
        add(P, X->x_cm, X->y_cm);
        return;
    }
    float s = chord2 / (2.0f * nc);
    float ox = E->x_cm + s * nx, oy = E->y_cm + s * ny, r = fabsf(s);
    float a0 = atan2f(E->y_cm - oy, E->x_cm - ox) * R2D;
    float a1 = atan2f(X->y_cm - oy, X->x_cm - ox) * R2D;
    float sweep = a1 - a0;
    if (s > 0.0f) { while (sweep < 0.0f)    sweep += 360.0f; while (sweep >= 360.0f) sweep -= 360.0f; }
    else          { while (sweep > 0.0f)    sweep -= 360.0f; while (sweep <= -360.0f) sweep += 360.0f; }
    add_arc(P, ox, oy, r, a0, sweep);
    add(P, X->x_cm, X->y_cm);
    (void)is_u;
}

void path_build(path_t *P) { path_build_from(P, WP_START_IDX); }

int path_nearest_wp(const pose_t *pose, float *dist_cm) {
    int best = 0; float bd = 1e9f;
    for (int i = 0; i < WP_COUNT; i++) {
        float d = hypotf(WAYPOINTS[i].x_cm - pose->x_cm, WAYPOINTS[i].y_cm - pose->y_cm);
        if (d < bd) { bd = d; best = i; }
    }
    if (dist_cm) *dist_cm = bd;
    return best;
}

float path_start_heading_error(const path_t *P, const pose_t *pose) {
    if (P->n < 2) return 0.0f;
    float b = atan2f(P->p[1].y - P->p[0].y, P->p[1].x - P->p[0].x) * R2D;
    return wrap180(b - pose->theta_deg);
}

void path_build_from(path_t *P, int from) {
    P->n = 0; P->seg = 0; P->done = false;
    int i = from;
    while (i < WP_COUNT) {
        const waypoint_t *w = &WAYPOINTS[i];
        if (i + 1 < WP_COUNT) {
            const waypoint_t *nx = &WAYPOINTS[i + 1];
            if (w->kind == WP_U_ENTRY && nx->kind == WP_U_EXIT)    { add_turn(P, w, nx, true);  i += 2; continue; }
            if (w->kind == WP_C90_ENTRY && nx->kind == WP_C90_EXIT) { add_turn(P, w, nx, false); i += 2; continue; }
        }
        add(P, w->x_cm, w->y_cm);
        i++;
    }
    // cierra la vuelta: P14 -> P1 (sube por la recta izquierda)
    add(P, WAYPOINTS[WP_FINAL_IDX].x_cm, WAYPOINTS[WP_FINAL_IDX].y_cm);
}

static int32_t clamp_speed(float v) {
    if (v >  6000.0f) return  6000;
    if (v < -6000.0f) return -6000;
    return (int32_t)lroundf(v);
}

// Proyeccion de (px,py) sobre el segmento s: devuelve t (0..1 dentro del segmento).
static float seg_t(const path_t *P, int s, float px, float py) {
    float ax = P->p[s].x, ay = P->p[s].y, bx = P->p[s + 1].x, by = P->p[s + 1].y;
    float dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    if (len2 < 1e-6f) return 1.0f;
    return ((px - ax) * dx + (py - ay) * dy) / len2;
}

void path_follow(path_t *P, const pose_t *pose, drive_cmd_t *out) {
    float px = pose->x_cm, py = pose->y_cm;
    const pt_t *end = &P->p[P->n - 1];
    out->dist_end_cm = hypotf(end->x - px, end->y - py);

    // 1) avanzar el segmento actual mientras la proyeccion ya paso su final
    while (P->seg < P->n - 2 && seg_t(P, P->seg, px, py) >= 1.0f) P->seg++;

    // proyeccion sobre el segmento actual y distancia a la ruta
    float t = seg_t(P, P->seg, px, py);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    pt_t cur = { P->p[P->seg].x + t * (P->p[P->seg + 1].x - P->p[P->seg].x),
                 P->p[P->seg].y + t * (P->p[P->seg + 1].y - P->p[P->seg].y) };
    out->xtrack_cm = hypotf(px - cur.x, py - cur.y);

    // 2) llego? (solo si ya va en los ultimos segmentos: P14 es inicio y final)
    if (P->seg >= P->n - 4 && out->dist_end_cm < FINISH_RADIUS_CM) P->done = true;
    if (P->done) { out->left = out->right = 0; out->err_deg = 0; return; }

    // 3) punto objetivo: avanzar LOOKAHEAD_CM sobre la ruta desde la proyeccion
    float rem = LOOKAHEAD_CM;
    pt_t target = cur;
    for (int s = P->seg; s < P->n - 1; s++) {
        pt_t b = P->p[s + 1];
        float d = hypotf(b.x - cur.x, b.y - cur.y);
        if (d >= rem && d > 1e-6f) {
            target.x = cur.x + (b.x - cur.x) * rem / d;
            target.y = cur.y + (b.y - cur.y) * rem / d;
            break;
        }
        rem -= d; cur = b; target = b;
    }

    // 4) error de rumbo y comandos de motor
    float bearing = atan2f(target.y - py, target.x - px) * R2D;
    float e = wrap180(bearing - pose->theta_deg);          // + = girar a la izquierda
    float ae = fabsf(e); if (ae > TURN_SLOW_DEG) ae = TURN_SLOW_DEG;
    float v = BASE_SPEED * (1.0f - 0.5f * ae / TURN_SLOW_DEG);
    float k = KP_TURN * e;
    out->left  = clamp_speed(v - k);
    out->right = clamp_speed(v + k);
    out->err_deg = e;
}
