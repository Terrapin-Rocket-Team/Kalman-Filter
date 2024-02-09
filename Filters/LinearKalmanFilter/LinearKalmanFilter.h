#include "../../MatrixMult/MatrixMult.h"

#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H

typedef struct {
    double* F; // State Transition Matrix
    double* G; // Control Matrix
    double* H; // Observation Matrix
    double* P; // Estimate Covariance Matrix
    double* R; // Measurement Uncertainty Matrix
    double* K; // Kalman Gain Matrix
    double* Q; // Process Noise Matrix
    double* U; // Control Vector
    double* X; // State Vector
    int n;     // State Vector Length
    int m;     // Measurement Vector Length
} KFState;

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control);
KFState predict_state(KFState state);
KFState estimate_state(KFState state, double *measurement);
KFState calculate_kalman_gain(KFState state);
KFState covariance_update(KFState state);
KFState covariance_extrapolate(KFState state);
KFState calculate_initial_values(KFState state, float dt);
KFState iterate(KFState state, float dt, double* measurement_vector, double* control_vector, bool has_gps);
#endif