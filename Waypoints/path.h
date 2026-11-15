// Ruta (arcos entre waypoints), perfil de velocidad y seguidor pure pursuit.
// Solo matematica, sin hardware: se puede simular en la PC.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "waypoints.h"

// Parametros de ajuste; se pueden sobreescribir con -D.
#ifndef STRAIGHT_SPEED
#define STRAIGHT_SPEED   2000    // velocidad en rectas, 0..6000
#endif
#ifndef CURVE_SPEED
#define CURVE_SPEED      1200    // velocidad en curvas
#endif
#ifndef KAPPA_REF
#define KAPPA_REF        0.08f   // curvatura (1/cm) a partir de la cual se usa CURVE_SPEED
#endif
#ifndef BRAKE_DIST_CM
#define BRAKE_DIST_CM    25.0f   // cuanto antes de la curva empieza a frenar
#endif
#ifndef ACCEL_STEP
#define ACCEL_STEP       250     // maximo aumento de velocidad por paso de control
#endif
#ifndef LOOKAHEAD_CM
#define LOOKAHEAD_CM     7.0f
#endif
#ifndef KP_TURN
#define KP_TURN          22.0f   // diferencia de motores por grado de error de rumbo
#endif
#ifndef TURN_SLOW_DEG
#define TURN_SLOW_DEG    50.0f   // con este error de rumbo la velocidad baja a la mitad
#endif
#ifndef TRACK_CM
#define TRACK_CM         9.5f    // separacion entre orugas
#endif
#ifndef FF_GAIN
#define FF_GAIN          0.5f    // ganancia del giro anticipado segun la curvatura
#endif
#ifndef FINISH_RADIUS_CM
#define FINISH_RADIUS_CM 6.0f    // distancia al punto final para detenerse
#endif
#ifndef LINE_FINISH_CM
#define LINE_FINISH_CM   2.5f    // idem para el tramo recto de la Parte A
#endif
#ifndef WP_REACH_CM
#define WP_REACH_CM      8.0f    // a esta distancia de un waypoint se considera alcanzado
#endif
#define PATH_MAX_POINTS  128
#define ARC_STEP_DEG     20.0f

typedef struct { float x, y; } pt_t;

typedef struct {
    pt_t  p[PATH_MAX_POINTS];
    float seg_kappa[PATH_MAX_POINTS];   // curvatura con signo del segmento (+ = antihorario); 0 en rectas
    int   n;
    int   seg;                          // segmento actual, de p[seg] a p[seg+1]; solo avanza
    float finish_cm;
    float v_cmd;                        // velocidad actual, para la rampa de aceleracion
    bool  done;

    // Seguimiento de waypoints, en el orden en que se recorren
    int    wp_seq[WP_COUNT];            // indices en WAYPOINTS[] en orden de recorrido
    int    wp_n;                        // cuantos waypoints tiene esta ruta
    int    wp_next;                     // posicion en wp_seq del siguiente por alcanzar
    int    wp_vertex[WP_COUNT];         // vertice de la ruta donde esta cada waypoint
    int8_t wp_state[WP_COUNT];          // 0 pendiente, 1 alcanzado, -1 omitido (por indice de WAYPOINTS[])
    float  wp_dist[WP_COUNT];           // distancia a la que se alcanzo (cm)
} path_t;

typedef struct {
    uint16_t wp_new_reached;   // bit k = WAYPOINTS[k] alcanzado en este paso
    uint16_t wp_new_missed;    // bit k = WAYPOINTS[k] omitido en este paso
    int32_t left, right;   // para motors_set_speeds()
    float   err_deg;       // error de rumbo; positivo = objetivo a la izquierda
    float   speed;         // velocidad base calculada
    float   dist_end_cm;
    float   xtrack_cm;     // distancia a la ruta
} drive_cmd_t;

// Ruta completa P1 -> ... -> P14 -> P1, con un arco entre cada par ENTRY/EXIT.
void path_build(path_t *P);

// Igual, pero saliendo del waypoint `from` (indice en WAYPOINTS[]) y terminando en P1.
void path_build_from(path_t *P, int from);

// Linea recta entre dos puntos (Parte A).
void path_build_line(path_t *P, float x0, float y0, float x1, float y1);

int path_nearest_wp(const pose_t *pose, float *dist_cm);

// Error de rumbo hacia el primer tramo de la ruta; positivo = girar a la izquierda.
float path_start_heading_error(const path_t *P, const pose_t *pose);

// Un paso de control con una pose nueva. Deja done = true al llegar al final.
void path_follow(path_t *P, const pose_t *pose, drive_cmd_t *out);

// Cuantos waypoints se han alcanzado y numero (P#) del siguiente; 1 (P1) cuando solo falta cerrar la vuelta.
int path_wp_reached(const path_t *P);
int path_next_wp_id(const path_t *P);
