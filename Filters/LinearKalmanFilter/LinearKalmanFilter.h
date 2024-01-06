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

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control){
    KFState state;

    state.X = initial_state;
    state.U = initial_control;

    calculate_initial_values(&state, 0.05);
    return state;
}

void predict_state(KFState *state){
    state->X = addMatrices(multiplyMatrices((*state).F, (*state).X, 6, 6, 6, 1), multiplyMatrices((*state).G, (*state).U, 6, 3, 3, 1), 6, 1);
}

void estimate_state(KFState *state, double *measurement){
    state->X = addMatrices((*state).X, multiplyMatrices((*state).K, subMatrices(measurement, multiplyMatrices((*state).H, (*state).X, 3, 6, 6, 1), 3, 1), 6, 3, 3, 1), 6, 1);
}

void calculate_kalman_gain(KFState *state){
    state->K = multiplyMatrices(
        multiplyMatrices(
            (*state).P, 
            transposeMatrix((*state).H, 3, 6), 
            6, 6, 6, 3
        ), 
        inverseMatrix(
            addMatrices(
                multiplyMatrices(
                    multiplyMatrices((*state).H, (*state).P, 3, 6, 6, 6), 
                    transposeMatrix((*state).H, 3, 6),
                    3, 6, 6, 3
                ), 
                (*state).R,
                3, 3
            ),
            3
        ),
        6, 3, 3, 3
    );
}

void covariance_update(KFState *state){
    state->P = addMatrices(
            multiplyMatrices(
                multiplyMatrices(
                    subMatrices(ident(6), multiplyMatrices((*state).K, (*state).H, 6, 3, 3, 6), 6, 6),
                    (*state).P,
                    6, 6, 6, 6
                ),
                transposeMatrix(
                    subMatrices(
                        ident(6),
                        multiplyMatrices((*state).K, (*state).H, 6, 3, 3, 6),
                        6, 6
                    ),
                    6, 6
                ),
                6, 6, 6, 6
            ),
            multiplyMatrices(
                multiplyMatrices((*state).K, (*state).R, 6, 3, 3, 3),
                transposeMatrix((*state).K, 6, 3),
                6, 3, 3, 6
            ),
            6, 6
        );
}

void covariance_extrapolate(KFState *state){
    state->P = addMatrices(
        multiplyMatrices(
            multiplyMatrices(
                (*state).F,
                (*state).P,
                6, 6, 6, 6
            ),
            transposeMatrix((*state).F, 6, 6),
            6, 6, 6, 6
        ),
        (*state).Q,
        6, 6
    );
}

void calculate_initial_values(KFState *state, float dt){
    state->F = new double[36]{1, 0, 0, dt, 0, 0,
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 0, 1};

    state->G = new double[18]{0.5*dt*dt, 0, 0,
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt};
    
    state->Q = multiplyMatrices((*state).G, multiplyByScalar(transposeMatrix((*state).G, 6, 3), 18, 0.2*0.2), 6, 3, 3, 6);

    predict_state(state);
    covariance_extrapolate(state);
}

double* iterate(KFState *state, float dt, double* measurement_vector, double* control_vector){
    // state->F = new double[36]{1, 0, 0, dt, 0, 0,
    //                         0, 1, 0, 0, dt, 0,
    //                         0, 0, 1, 0, 0, dt,
    //                         0, 0, 0, 1, 0, 0,
    //                         0, 0, 0, 0, 0, 1};

    // state->G = new double[18]{0.5*dt*dt, 0, 0,
    //                         0, 0.5*dt*dt, 0,
    //                         0, 0, 0.5*dt*dt,
    //                         dt, 0, 0,
    //                         0, dt, 0,
    //                         0, 0, dt};
    
    state->Q = multiplyMatrices((*state).G, multiplyByScalar(transposeMatrix((*state).G, 6, 3), 18, 1.5*1.5), 6, 3, 3, 6);

    state->R = new double[9]{1.5, 0, 0,
                            0, 1.5, 0,
                            0, 0, 1.5};

    state->H = new double[18]{1, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0};
    

    calculate_kalman_gain(state);
    estimate_state(state, measurement_vector);
    covariance_update(state);

    predict_state(state);
    covariance_extrapolate(state);

    delete[] state->Q;
    delete[] state->R;
    delete[] state->H;

    return (*state).X;
}

#endif
