#include "MatrixMult.h"

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

} KFState;

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control);

void predict_state(KFState *state);

void estimate_state(KFState *state, double *measurement);

void calculate_kalman_gain(KFState *state);

void covariance_update(KFState *state);

void covariance_extrapolate(KFState *state);

void calculate_initial_values(KFState *state, float dt);

double* iterate(KFState *state, float dt, double* measurement_vector, double* control_vector);

#endif
