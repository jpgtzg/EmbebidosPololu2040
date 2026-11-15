// Imprime la ruta deseada (la misma que usa el Zumo) y los waypoints, en CSV.
// gcc -I../Waypoints dump_route.c ../Waypoints/path.c ../Waypoints/waypoints.c -lm -o dump_route
#include <stdio.h>
#include "path.h"

int main(int argc, char **argv) {
    static path_t P;
    path_build(&P);
    FILE *f = fopen(argc > 1 ? argv[1] : "route.csv", "w");
    fprintf(f, "x_cm,y_cm\n");
    for (int i = 0; i < P.n; i++) fprintf(f, "%.2f,%.2f\n", P.p[i].x, P.p[i].y);
    fclose(f);
    f = fopen(argc > 2 ? argv[2] : "waypoints.csv", "w");
    fprintf(f, "id,x_cm,y_cm,theta_deg\n");
    for (int i = 0; i < WP_COUNT; i++)
        fprintf(f, "%d,%.2f,%.2f,%.1f\n", WAYPOINTS[i].id, WAYPOINTS[i].x_cm, WAYPOINTS[i].y_cm, WAYPOINTS[i].theta_deg);
    fclose(f);
    printf("ruta: %d puntos\n", P.n);
    return 0;
}
