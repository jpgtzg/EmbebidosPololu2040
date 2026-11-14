// Construccion de la ruta a partir de los waypoints, perfil de velocidad y control de
// seguimiento (pure pursuit con giro anticipado por curvatura).
#include <math.h>
#include "path.h"

#define D2R 0.01745329252f
#define R2D 57.2957795f

#define ARC_SEG_MAX_CM  12.0f   // segmentos mas largos se consideran recta
#define END_MIN_SPEED   500
#define END_BRAKE_PER_CM 60.0f
#define PRED_ALPHA      0.5f
#define PRED_MAX_MS     300.0f

static float wrap180(float a) {
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static void add(path_t *P, float x, float y) {
    if (P->n < PATH_MAX_POINTS) { P->p[P->n].x = x; P->p[P->n].y = y; P->n++; }
}

// Puntos interiores de un arco; sweep en grados, positivo = antihorario.
static void add_arc(path_t *P, float cx, float cy, float r, float a0, float sweep) {
    int steps = (int)ceilf(fabsf(sweep) / ARC_STEP_DEG);
    for (int k = 1; k < steps; k++) {
        float a = (a0 + sweep * k / steps) * D2R;
        add(P, cx + r * cosf(a), cy + r * sinf(a));
    }
}

// Agrega la entrada E, el arco y la salida X. El arco es el circulo tangente al rumbo de
// entrada que pasa por E y por X.
static void add_turn(path_t *P, const waypoint_t *E, const waypoint_t *X) {
    add(P, E->x_cm, E->y_cm);

    float hx = cosf(E->theta_deg * D2R), hy = sinf(E->theta_deg * D2R);
    float cx = X->x_cm - E->x_cm, cy = X->y_cm - E->y_cm;
    float chord2 = cx * cx + cy * cy;
    float chord = sqrtf(chord2);

    float nx = -hy, ny = hx;
    float nc = nx * cx + ny * cy;
    if (chord < 6.0f || fabsf(nc) < 0.02f * chord) {
        add(P, X->x_cm, X->y_cm);
        return;
    }

    float s = chord2 / (2.0f * nc);   // radio con signo: positivo = centro a la izquierda
    float ox = E->x_cm + s * nx, oy = E->y_cm + s * ny;
    float a0 = atan2f(E->y_cm - oy, E->x_cm - ox) * R2D;
    float a1 = atan2f(X->y_cm - oy, X->x_cm - ox) * R2D;
    float sweep = a1 - a0;
    if (s > 0.0f) { while (sweep < 0.0f)  sweep += 360.0f; while (sweep >= 360.0f)  sweep -= 360.0f; }
    else          { while (sweep > 0.0f)  sweep -= 360.0f; while (sweep <= -360.0f) sweep += 360.0f; }
    add_arc(P, ox, oy, fabsf(s), a0, sweep);
    add(P, X->x_cm, X->y_cm);
}

// Curvatura con signo de cada segmento (1/cm), a partir del cambio de rumbo en sus vertices.
static void finalize(path_t *P, float finish_cm) {
    P->seg = 0; P->done = false; P->finish_cm = finish_cm; P->v_cmd = CURVE_SPEED * 0.5f;

    static float vk[PATH_MAX_POINTS];
    for (int j = 0; j < PATH_MAX_POINTS; j++) vk[j] = 0.0f;
    for (int j = 1; j < P->n - 1; j++) {
        float ax = P->p[j].x - P->p[j - 1].x, ay = P->p[j].y - P->p[j - 1].y;
        float bx = P->p[j + 1].x - P->p[j].x, by = P->p[j + 1].y - P->p[j].y;
        float la = hypotf(ax, ay), lb = hypotf(bx, by);
        if (la < 1e-3f || lb < 1e-3f) continue;
        float da = wrap180((atan2f(by, bx) - atan2f(ay, ax)) * R2D);
        vk[j] = da * D2R / (la < lb ? la : lb);
    }
    for (int s = 0; s < PATH_MAX_POINTS; s++) P->seg_kappa[s] = 0.0f;
    for (int s = 0; s < P->n - 1; s++) {
        float len = hypotf(P->p[s + 1].x - P->p[s].x, P->p[s + 1].y - P->p[s].y);
        P->seg_kappa[s] = len > ARC_SEG_MAX_CM ? 0.0f : 0.5f * (vk[s] + vk[s + 1]);
    }
}

void path_build(path_t *P) { path_build_from(P, WP_START_IDX); }

void path_build_from(path_t *P, int from) {
    P->n = 0;
    int i = from;
    while (i < WP_COUNT) {
        const waypoint_t *w = &WAYPOINTS[i];
        if (i + 1 < WP_COUNT) {
            const waypoint_t *nx = &WAYPOINTS[i + 1];
            bool u   = w->kind == WP_U_ENTRY   && nx->kind == WP_U_EXIT;
            bool c90 = w->kind == WP_C90_ENTRY && nx->kind == WP_C90_EXIT;
            if (u || c90) { add_turn(P, w, nx); i += 2; continue; }
        }
        add(P, w->x_cm, w->y_cm);
        i++;
    }
    add(P, WAYPOINTS[WP_FINAL_IDX].x_cm, WAYPOINTS[WP_FINAL_IDX].y_cm);
    finalize(P, FINISH_RADIUS_CM);
}

void path_build_line(path_t *P, float x0, float y0, float x1, float y1) {
    P->n = 0;
    add(P, x0, y0);
    add(P, x1, y1);
    finalize(P, LINE_FINISH_CM);
}

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

static int32_t clamp_speed(float v) {
    if (v >  6000.0f) return  6000;
    if (v < -6000.0f) return -6000;
    return (int32_t)lroundf(v);
}

// Parametro t (0..1 dentro del segmento) de la proyeccion de (px, py) sobre el segmento s.
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

    while (P->seg < P->n - 2 && seg_t(P, P->seg, px, py) >= 1.0f) P->seg++;

    float t = seg_t(P, P->seg, px, py);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    pt_t cur = { P->p[P->seg].x + t * (P->p[P->seg + 1].x - P->p[P->seg].x),
                 P->p[P->seg].y + t * (P->p[P->seg + 1].y - P->p[P->seg].y) };
    out->xtrack_cm = hypotf(px - cur.x, py - cur.y);

    // Inicio y final coinciden en P1: solo cuenta como llegada en los ultimos segmentos.
    bool final_region = P->seg >= P->n - 4;
    if (final_region && out->dist_end_cm < P->finish_cm) P->done = true;
    if (P->done) { out->left = out->right = 0; out->err_deg = 0; out->speed = 0; return; }

    // Punto objetivo: LOOKAHEAD_CM adelante sobre la ruta.
    float rem = LOOKAHEAD_CM;
    pt_t target = cur, c = cur;
    int tseg = P->seg;
    for (int s = P->seg; s < P->n - 1; s++) {
        tseg = s;
        pt_t b = P->p[s + 1];
        float d = hypotf(b.x - c.x, b.y - c.y);
        if (d >= rem && d > 1e-6f) {
            target.x = c.x + (b.x - c.x) * rem / d;
            target.y = c.y + (b.y - c.y) * rem / d;
            break;
        }
        rem -= d; c = b; target = b;
    }

    // Velocidad: rapido en rectas, lento si hay una curva dentro de BRAKE_DIST_CM.
    float kmax = 0.0f, walked = 0.0f;
    pt_t w = cur;
    for (int s = P->seg; s < P->n - 1 && walked < BRAKE_DIST_CM; s++) {
        float k = fabsf(P->seg_kappa[s]);
        if (k > kmax) kmax = k;
        walked += hypotf(P->p[s + 1].x - w.x, P->p[s + 1].y - w.y);
        w = P->p[s + 1];
    }
    float f = 1.0f - kmax / KAPPA_REF;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    float v = CURVE_SPEED + (STRAIGHT_SPEED - CURVE_SPEED) * f;

    float bearing = atan2f(target.y - py, target.x - px) * R2D;
    float e = wrap180(bearing - pose->theta_deg);
    float ae = fabsf(e); if (ae > TURN_SLOW_DEG) ae = TURN_SLOW_DEG;
    v *= 1.0f - 0.5f * ae / TURN_SLOW_DEG;

    if (final_region) {
        float vend = END_MIN_SPEED + END_BRAKE_PER_CM * out->dist_end_cm;
        if (v > vend) v = vend;
    }
    if (v > P->v_cmd + ACCEL_STEP) v = P->v_cmd + ACCEL_STEP;
    P->v_cmd = v;

    // Giro anticipado: diferencia de ruedas que da la curvatura del tramo objetivo.
    float ff = FF_GAIN * P->seg_kappa[tseg] * TRACK_CM * v * 0.5f;
    float k = KP_TURN * e + ff;
    out->left  = clamp_speed(v - k);
    out->right = clamp_speed(v + k);
    out->err_deg = e;
    out->speed = v;
}

void pred_reset(pred_t *q) { q->have = false; q->vx = q->vy = q->w = 0.0f; }

void pred_update(pred_t *q, const pose_t *pose, uint32_t t_ms) {
    if (q->have) {
        float dt = (float)(t_ms - q->t_ms) / 1000.0f;
        if (dt > 0.02f && dt < 0.5f) {
            q->vx = PRED_ALPHA * ((pose->x_cm - q->x) / dt) + (1.0f - PRED_ALPHA) * q->vx;
            q->vy = PRED_ALPHA * ((pose->y_cm - q->y) / dt) + (1.0f - PRED_ALPHA) * q->vy;
            q->w  = PRED_ALPHA * (wrap180(pose->theta_deg - q->th) / dt) + (1.0f - PRED_ALPHA) * q->w;
        } else {
            q->vx = q->vy = q->w = 0.0f;
        }
    }
    q->x = pose->x_cm; q->y = pose->y_cm; q->th = pose->theta_deg;
    q->t_ms = t_ms; q->have = true;
}

void pred_apply(const pred_t *q, const pose_t *pose, float horizon_ms, pose_t *out) {
    if (horizon_ms < 0.0f) horizon_ms = 0.0f;
    if (horizon_ms > PRED_MAX_MS) horizon_ms = PRED_MAX_MS;
    float h = horizon_ms / 1000.0f;
    out->x_cm = pose->x_cm + q->vx * h;
    out->y_cm = pose->y_cm + q->vy * h;
    out->theta_deg = wrap180(pose->theta_deg + q->w * h);
}
