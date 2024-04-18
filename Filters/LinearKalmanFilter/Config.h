#ifndef CONFIG_H
#define CONFIG_H

#include "../../MatrixMult/Matrix.h"

typedef struct {
    Matrix X, U, P, F, G, R, H;
} Config;

Config initialize_filter();
Config iterate_filter();

#endif