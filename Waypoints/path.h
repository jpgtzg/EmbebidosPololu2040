// Ruta (arcos entre waypoints) y seguidor pure pursuit. Solo matematica, sin hardware.
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "waypoints.h"

// Parametros de ajuste; se pueden sobreescribir con -D.
#ifndef LOOKAHEAD_CM
#define LOOKAHEAD_CM     7.0f    // distancia al punto objetivo sobre la ruta
#endif
#ifndef BASE_SPEED
#define BASE_SPEED       1500    // 0..6000
#endif
#ifndef KP_TURN
#define KP_TURN          22.0f   // diferencia de motores por grado de error de rumbo
#endif
#ifndef TURN_SLOW_DEG
#define TURN_SLOW_DEG    50.0f   // con este error de rumbo la velocidad baja a la mitad
#endif
#ifndef FINISH_RADIUS_CM
#define FINISH_RADIUS_CM 6.0f    // distancia al punto final para detenerse
#endif
#define PATH_MAX_POINTS  128
#define ARC_STEP_DEG     20.0f

typedef struct { float x, y; } pt_t;

typedef struct {
    pt_t p[PATH_MAX_POINTS];
    int  n;
    int  seg;     // segmento actual, de p[seg] a p[seg+1]; solo avanza
    bool done;
} path_t;

typedef struct {
    int32_t left, right;   // para motors_set_speeds()
    float   err_deg;       // error de rumbo; positivo = objetivo a la izquierda
    float   dist_end_cm;
    float   xtrack_cm;     // distancia a la ruta
} drive_cmd_t;

// Ruta completa P1 -> ... -> P14 -> P1, con un arco entre cada par ENTRY/EXIT.
void path_build(path_t *P);

// Igual, pero saliendo del waypoint `from` (indice en WAYPOINTS[]) y terminando en P1.
void path_build_from(path_t *P, int from);

int path_nearest_wp(const pose_t *pose, float *dist_cm);

// Error de rumbo hacia el primer tramo de la ruta; positivo = girar a la izquierda.
float path_start_heading_error(const path_t *P, const pose_t *pose);

// Un paso de control con una pose nueva. Deja done = true al llegar al final.
void path_follow(path_t *P, const pose_t *pose, drive_cmd_t *out);
