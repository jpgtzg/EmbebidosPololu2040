// Angulo alfa entre la fila de sensores y la tangente al arco de gris que pasa por S3.
#pragma once
#include <stdbool.h>

#define ALPHA_SENSORS 5

// Punto de calibracion: tiempo de descarga (us) medido a un radio conocido (m).
typedef struct { float t_us; float radius_m; } calib_pt_t;

// Radio a partir del tiempo de descarga, por interpolacion lineal en la tabla.
// La tabla debe estar ordenada por t_us creciente; fuera de rango usa el extremo.
float radius_from_time(const calib_pt_t *tab, int n, float t_us);

// r[i]: radio de cada sensor al vertice blanco (m). s[i]: posicion sobre la fila (m), S3 = 0.
// Ajusta por minimos cuadrados  r_i^2 - s_i^2 = r3^2 + 2*r3*sin(alpha)*s_i  y devuelve
// alpha (grados, de -90 a 90) y el radio en S3. false si los datos no son validos.
bool alpha_from_radii(const float r[ALPHA_SENSORS], const float s[ALPHA_SENSORS],
                      float *alpha_deg, float *r3_m);
