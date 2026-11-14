#include <math.h>
#include "alpha.h"

float radius_from_time(const calib_pt_t *tab, int n, float t_us) {
    if (n < 2) return n == 1 ? tab[0].radius_m : 0.0f;
    if (t_us <= tab[0].t_us) return tab[0].radius_m;
    for (int i = 1; i < n; i++) {
        if (t_us <= tab[i].t_us) {
            float u = (t_us - tab[i - 1].t_us) / (tab[i].t_us - tab[i - 1].t_us);
            return tab[i - 1].radius_m + u * (tab[i].radius_m - tab[i - 1].radius_m);
        }
    }
    return tab[n - 1].radius_m;
}

bool alpha_from_radii(const float r[ALPHA_SENSORS], const float s[ALPHA_SENSORS],
                      float *alpha_deg, float *r3_m) {
    float ms = 0.0f, my = 0.0f, y[ALPHA_SENSORS];
    for (int i = 0; i < ALPHA_SENSORS; i++) {
        y[i] = r[i] * r[i] - s[i] * s[i];
        ms += s[i];
        my += y[i];
    }
    ms /= ALPHA_SENSORS;
    my /= ALPHA_SENSORS;

    float sxx = 0.0f, sxy = 0.0f;
    for (int i = 0; i < ALPHA_SENSORS; i++) {
        sxx += (s[i] - ms) * (s[i] - ms);
        sxy += (s[i] - ms) * (y[i] - my);
    }
    if (sxx < 1e-12f) return false;

    float b = sxy / sxx;            // 2 * r3 * sin(alpha)
    float a = my - b * ms;          // r3^2, el valor de la recta en s = 0
    if (a <= 0.0f) return false;

    float r3 = sqrtf(a);
    float sa = b / (2.0f * r3);
    if (sa > 1.0f) sa = 1.0f;
    if (sa < -1.0f) sa = -1.0f;
    *alpha_deg = asinf(sa) * 57.2957795f;
    *r3_m = r3;
    return true;
}
