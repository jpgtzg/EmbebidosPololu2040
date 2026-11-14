// path.h - Part C: ruta (arcos entre waypoints) y seguidor "pure pursuit".
// Solo matematica: no usa hardware, por eso se puede simular en la PC.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "waypoints.h"

// ---- parametros de ajuste (se pueden sobreescribir con -D en CMake) ----
#ifndef LOOKAHEAD_CM
#define LOOKAHEAD_CM     7.0f    // distancia al punto objetivo sobre la ruta
#endif
#ifndef BASE_SPEED
#define BASE_SPEED       1500    // 0..6000 (MOTORS_MAX_SPEED). Empieza lento.
#endif
#ifndef KP_TURN
#define KP_TURN          22.0f   // diferencia de motores por grado de error de rumbo
#endif
#ifndef TURN_SLOW_DEG
#define TURN_SLOW_DEG    50.0f   // con este error de rumbo va a la mitad de BASE_SPEED
#endif
#ifndef FINISH_RADIUS_CM
#define FINISH_RADIUS_CM 6.0f    // distancia a P14 para considerar que llego
#endif
#define PATH_MAX_POINTS  128
#define ARC_STEP_DEG     20.0f

typedef struct { float x, y; } pt_t;

typedef struct {
    pt_t p[PATH_MAX_POINTS];
    int  n;       // numero de puntos
    int  seg;     // segmento actual (de p[seg] a p[seg+1]); solo avanza
    bool done;
} path_t;

typedef struct {
    int32_t left, right;   // comandos para motors_set_speeds()
    float   err_deg;       // error de rumbo (+ = objetivo a la izquierda)
    float   dist_end_cm;   // distancia al punto final
    float   xtrack_cm;     // distancia a la ruta (para detectar que se salio)
} drive_cmd_t;

// Construye la ruta P14 -> P1 -> ... -> P13 -> P14 a partir de WAYPOINTS[]:
// entre cada ENTRY/EXIT agrega un arco (semicirculo en las U, cuarto de circulo en
// las esquinas de 90 grados).
void path_build(path_t *P);

// Igual, pero empezando en el waypoint `from` (indice 0..13 de WAYPOINTS[]) y terminando
// en P14. from = WP_START_IDX (P14) da la vuelta completa.
void path_build_from(path_t *P, int from);

// Waypoint mas cercano a la pose (indice) y su distancia en cm.
int path_nearest_wp(const pose_t *pose, float *dist_cm);

// Error de rumbo (grados, + = girar a la izquierda) hacia el primer tramo de la ruta.
float path_start_heading_error(const path_t *P, const pose_t *pose);

// Un paso del control con una pose NUEVA de la camara. Deja done=true al llegar al final.
void path_follow(path_t *P, const pose_t *pose, drive_cmd_t *out);
