#include "LinearKalmanFilter.h"

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control){
    KFState state;

    state.X = initial_state;
    state.U = initial_control;
    state.P = new double[36]{100, 0, 0, 0, 0, 0,
                            0, 100, 0, 0, 0, 0,
                            0, 0, 100, 0, 0, 0,
                            0, 0, 0, 100, 0, 0,
                            0, 0, 0, 0, 100, 0,
                            0, 0, 0, 0, 0, 100};
    state.K = new double;
    state.Q = new double;
    state = calculate_initial_values(state, 0.05);
    return state;
}

KFState predict_state(KFState state){
    double* inter1 = multiplyMatrices(state.F, state.X, 6, 6, 6, 1);
    double* inter2 = multiplyMatrices(state.G, state.U, 6, 3, 3, 1);
    delete[] state.X;
    state.X = addMatrices(inter1, inter2, 6, 1);

    delete[] inter1;
    delete[] inter2;

    return state;
}

KFState estimate_state(KFState state, double *measurement){
    double* inter1 = multiplyMatrices(state.H, state.X, 3, 6, 6, 1);
    double* inter2 = subMatrices(measurement, inter1, 3, 1);
    double* inter3 = multiplyMatrices(state.K, inter2, 6, 3, 3, 1);
    double* inter4 = state.X;
    state.X = addMatrices(inter4, inter3, 6, 1);

    delete[] inter1;
    delete[] inter2;
    delete[] inter3;
    delete[] inter4;

    return state;
}

KFState calculate_kalman_gain(KFState state){
    double* inter1 = transposeMatrix(state.H, 3, 6);
    double* inter2 = multiplyMatrices(state.P, inter1, 6, 6, 6, 3);
    double* inter3 = multiplyMatrices(state.H, state.P, 3, 6, 6, 6);
    double* inter4 = transposeMatrix(state.H, 3, 6);
    double* inter5 = multiplyMatrices(inter3, inter4, 3, 6, 6, 3);
    double* inter6 = addMatrices(inter5, state.R, 3, 3);
    double* inter7 = inverseMatrix(inter6, 3);
    delete[] state.K;
    state.K = multiplyMatrices(inter2, inter7, 6, 3, 3, 3);

    delete[] inter1;
    delete[] inter2;
    delete[] inter3;
    delete[] inter4;
    delete[] inter5;
    delete[] inter6;
    delete[] inter7;

    return state;
}

KFState covariance_update(KFState state){
    // state.P = addMatrices(
    //         multiplyMatrices(
    //             multiplyMatrices(
    //                 subMatrices(ident(6), multiplyMatrices(state.K, state.H, 6, 3, 3, 6), 6, 6),
    //                 state.P,
    //                 6, 6, 6, 6
    //             ),
    //             transposeMatrix(
    //                 subMatrices(
    //                     ident(6),
    //                     multiplyMatrices(state.K, state.H, 6, 3, 3, 6),
    //                     6, 6
    //                 ),
    //                 6, 6
    //             ),
    //             6, 6, 6, 6
    //         ),
    //         multiplyMatrices(
    //             multiplyMatrices(state.K, state.R, 6, 3, 3, 3),
    //             transposeMatrix(state.K, 6, 3),
    //             6, 3, 3, 6
    //         ),
    //         6, 6
    //     );
        double* inter1 = ident(6);
        double* inter2 = multiplyMatrices(state.K, state.H, 6, 3, 3, 6);
        double* inter3 = subMatrices(inter1, inter2, 6, 6);
        double* inter4 = transposeMatrix(inter3, 6, 6);
        double* inter5 = multiplyMatrices(inter3, state.P, 6, 6, 6, 6);
        double* inter6 = multiplyMatrices(inter5, inter4, 6, 6, 6, 6);
        double* inter7 = multiplyMatrices(state.K, state.R, 6, 3, 3, 3);
        double* inter8 = transposeMatrix(state.K, 6, 3);
        double* inter9 = multiplyMatrices(inter7, inter8, 6, 3, 3, 6);
        delete[] state.P;
        state.P = addMatrices(inter6, inter9, 6, 6);
        delete[] inter1;
        delete[] inter2;
        delete[] inter3;
        delete[] inter4;
        delete[] inter5;
        delete[] inter6;
        delete[] inter7;
        delete[] inter8;
        delete[] inter9;
        return state;
}

KFState covariance_extrapolate(KFState state){
    // state.P = addMatrices(
    //     multiplyMatrices(
    //         multiplyMatrices(
    //             state.F,
    //             state.P,
    //             6, 6, 6, 6
    //         ),
    //         transposeMatrix(state.F, 6, 6),
    //         6, 6, 6, 6
    //     ),
    //     state.Q,
    //     6, 6
    // );

    double* inter1 = transposeMatrix(state.F, 6, 6);
    double* inter2 = multiplyMatrices(state.F, state.P, 6, 6, 6, 6);
    double* inter3 = multiplyMatrices(inter2, inter1, 6, 6, 6, 6);
    delete[] state.P;
    state.P = addMatrices(inter3, state.Q, 6, 6);
    delete[] inter1;
    delete[] inter2;
    delete[] inter3;
    return state;
}

KFState calculate_initial_values(KFState state, float dt){
    state.F = new double[36]{1, 0, 0, dt, 0, 0,
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1};

    state.G = new double[18]{0.5*dt*dt, 0, 0,
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt};
    //state.Q = multiplyMatrices(state.G, multiplyByScalar(transposeMatrix(state.G, 6, 3), 18, 0.2*0.2), 6, 3, 3, 6);
    double* inter1 = transposeMatrix(state.G, 6, 3);
    double* inter2 = multiplyByScalar(inter1, 18, 0.2*0.2);
    delete[] state.Q;
    state.Q = multiplyMatrices(state.G, inter2, 6, 3, 3, 6);
    delete[] inter1;
    delete[] inter2;

    predict_state(state);
    covariance_extrapolate(state);
    return state;
}

KFState iterate(KFState state, float dt, double* measurement_vector, double* control_vector, bool has_gps){
    state.F = new double[36]{1, 0, 0, dt, 0, 0,
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1};

    state.G = new double[18]{0.5*dt*dt, 0, 0,
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt};

    if (has_gps) {
        state.H = new double[18]{1, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0};
    } else {
        state.H = new double[18]{0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0};
    }

    //state.Q = multiplyMatrices(state.G, multiplyByScalar(transposeMatrix(state.G, 6, 3), 18, 1.5*1.5), 6, 3, 3, 6);
    double* inter1 = transposeMatrix(state.G, 6, 3);
    double* inter2 = multiplyByScalar(inter1, 18, 1.5*1.5);
    delete[] state.Q;
    state.Q = multiplyMatrices(state.G, inter2, 6, 3, 3, 6);
    delete[] inter1;
    delete[] inter2;

    state.R = new double[9]{1.5, 0, 0,
                            0, 1.5, 0,
                            0, 0, 1.5};
    
    state = calculate_kalman_gain(state);
    state = estimate_state(state, measurement_vector);
    state = covariance_update(state);

    state = predict_state(state);
    state = covariance_extrapolate(state);

    delete[] state.F;
    delete[] state.G;
    delete[] state.H;
    delete[] state.R;

    return state;
}