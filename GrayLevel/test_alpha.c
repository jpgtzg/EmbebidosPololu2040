// Prueba en la PC: gcc test_alpha.c alpha.c -lm -o test_alpha
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "alpha.h"

static float noise(float amp) { return amp * ((float)rand() / RAND_MAX * 2.0f - 1.0f); }

int main(void) {
    const float s[5] = { -0.030f, -0.015f, 0.0f, 0.015f, 0.030f };

    // Ejemplo del enunciado: alpha = 30 grados
    float r_pdf[5] = { 0.1082f, 0.1132f, 0.1200f, 0.1282f, 0.1375f };
    float a, r3;
    if (alpha_from_radii(r_pdf, s, &a, &r3)) printf("Ejemplo del PDF: alpha=%.2f deg, r3=%.4f m (esperado 30)\n", a, r3);

    // Radios sinteticos para varios alpha y radio r3 = 1.00 m, con y sin ruido
    float alphas[] = { 0, 15, 30, 45, 60, 75, 90, -30 };
    for (float amp = 0.0f; amp <= 0.002f; amp += 0.001f) {
        printf("\nRuido en el radio: +-%.1f mm\n alpha real | alpha estimado\n", amp * 1000);
        for (int k = 0; k < 8; k++) {
            float al = alphas[k] * 0.01745329f, r[5];
            for (int i = 0; i < 5; i++) r[i] = sqrtf(1.0f + 2.0f * 1.0f * s[i] * sinf(al) + s[i] * s[i]) + noise(amp);
            if (alpha_from_radii(r, s, &a, &r3)) printf("  %6.1f    |  %6.1f   (r3=%.4f)\n", alphas[k], a, r3);
        }
    }
    return 0;
}
