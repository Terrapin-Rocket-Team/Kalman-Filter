#ifndef CONFIG_H
#define CONFIG_H

#include "../../MatrixMult/Matrix.h"

typedef struct {
    Matrix X, U, P, F, G, R, H;
} Config;

Config initialize_filter(double dt);
Config iterate_filter(double dt, double* meas_arr, double* control_arr, int num_satellites);

#endif