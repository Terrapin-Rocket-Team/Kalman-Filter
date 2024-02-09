#include "LinearKalmanFilter.h"

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control){
    KFState state;
    state.n = statevector_size;
    state.m = measurement_size;
    state.X = initial_state;
    state.U = initial_control;
    state.P = 100*eye(state.n); //TODO Create this function, TODO make the 100 a changable parameter
    state = calculate_initial_values(state, 0.05);
    return state;
}
KFState predict_state(KFState state){
    double* inter1 = multiplyMatrices(state.F, state.X, state.n, state.n, state.n, 1);
    double* inter2 = multiplyMatrices(state.G, state.U, state.n, state.m, state.m, 1);
    //delete[] state.X;
    state.X = addMatrices(inter1, inter2, 6, 1);
    delete[] inter1;
    delete[] inter2;
    return state;
}
KFState estimate_state(KFState state, double *measurement){
    double* inter1 = multiplyMatrices(state.H, state.X, state.m, state.n, state.n, 1);
    double* inter2 = subMatrices(measurement, inter1, state.m, 1);
    double* inter3 = multiplyMatrices(state.K, inter2, state.n, state.m, state.m, 1);
    double* inter4 = state.X;
    state.X = addMatrices(inter4, inter3, state.n, 1);
    delete[] inter1;
    delete[] inter2;
    delete[] inter3;
    delete[] inter4;
    return state;
}
KFState calculate_kalman_gain(KFState state){
    double* inter1 = transposeMatrix(state.H, state.m, state.n);
    double* inter2 = multiplyMatrices(state.P, inter1, state.n, state.n, state.n, state.m);
    double* inter3 = multiplyMatrices(state.H, state.P, state.m, state.n, state.n, state.n);
    double* inter4 = transposeMatrix(state.H, state.m, state.n);
    double* inter5 = multiplyMatrices(inter3, inter4, state.m, state.n, state.n, state.m);
    double* inter6 = addMatrices(inter5, state.R, state.m, state.m);
    double* inter7 = inverseMatrix(inter6, state.m);
    delete[] state.K;
    state.K = multiplyMatrices(inter2, inter7, state.n, state.m, state.m, state.m);
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
    //                 subMatrices(ident(state.n), multiplyMatrices(state.K, state.H, state.n, state.m, state.m, state.n), state.n, state.n),
    //                 state.P,
    //                 state.n, state.n, state.n, state.n
    //             ),
    //             transposeMatrix(
    //                 subMatrices(
    //                     ident(state.n),
    //                     multiplyMatrices(state.K, state.H, state.n, state.m, state.m, state.n),
    //                     state.n, state.n
    //                 ),
    //                 state.n, state.n
    //             ),
    //             state.n, state.n, state.n, state.n
    //         ),
    //         multiplyMatrices(
    //             multiplyMatrices(state.K, state.R, state.n, state.m, state.m, state.m),
    //             transposeMatrix(state.K, state.n, state.m),
    //             state.n, state.m, state.m, state.n
    //         ),
    //         state.n, state.n
    //     );
        double* inter1 = ident(state.n);
        double* inter2 = multiplyMatrices(state.K, state.H, state.n, state.m, state.m, state.n);
        double* inter3 = subMatrices(inter1, inter2, state.n, state.n);
        double* inter4 = transposeMatrix(inter3, state.n, state.n);
        double* inter5 = multiplyMatrices(inter3, state.P, state.n, state.n, state.n, state.n);
        double* inter6 = multiplyMatrices(inter5, inter4, state.n, state.n, state.n, state.n);
        double* inter7 = multiplyMatrices(state.K, state.R, state.n, state.m, state.m, state.m);
        double* inter8 = transposeMatrix(state.K, state.n, state.m);
        double* inter9 = multiplyMatrices(inter7, inter8, state.n, state.m, state.m, state.n);
        delete[] state.P;
        state.P = addMatrices(inter6, inter9, state.n, state.n);
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
    //             state.n, state.n, state.n, state.n
    //         ),
    //         transposeMatrix(state.F, state.n, state.n),
    //         state.n, state.n, state.n, state.n
    //     ),
    //     state.Q,
    //     state.n, state.n
    // );
    double* inter1 = transposeMatrix(state.F, state.n, state.n);
    double* inter2 = multiplyMatrices(state.F, state.P, state.n, state.n, state.n, state.n);
    double* inter3 = multiplyMatrices(inter2, inter1, state.n, state.n, state.n, state.n);
    //delete[] state.P;
    state.P = addMatrices(inter3, state.Q, state.n, state.n);
    delete[] inter1;
    delete[] inter2;
    delete[] inter3;
    return state;
}
KFState calculate_initial_values(KFState state, float dt){
    state.F = new double[36]{1, 0, 0, dt, 0, 0, // TODO hardcoded
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1};
    state.G = new double[18]{0.5*dt*dt, 0, 0, // TODO hardcoded
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt};
    //state.Q = multiplyMatrices(state.G, multiplyByScalar(transposeMatrix(state.G, state.n, state.m), 18, 0.2*0.2), state.n, state.m, state.m, state.n);
    double* inter1 = transposeMatrix(state.G, state.n, state.m);
    double* inter2 = multiplyByScalar(inter1, state.n*state.m, 0.2*0.2); // TODO make this a variable to tune
    //delete[] state.Q;
    state.Q = multiplyMatrices(state.G, inter2, state.n, state.m, state.m, state.n);
    delete[] inter1;
    delete[] inter2;
    predict_state(state);
    covariance_extrapolate(state);
    return state;
}
KFState iterate(KFState state, float dt, double* measurement_vector, double* control_vector, bool has_gps){
    state.F = new double[36]{1, 0, 0, dt, 0, 0, // TODO hardcoded
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1};
    state.G = new double[18]{0.5*dt*dt, 0, 0, // TODO hardcoded
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt};
    if (has_gps) {
        state.H = new double[18]{1, 0, 0, 0, 0, 0, // TODO hardcoded
                            0, 1, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0};
    } else {
        state.H = new double[18]{0, 0, 0, 0, 0, 0, // TODO hardcoded
                            0, 0, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0};
    }
    //state.Q = multiplyMatrices(state.G, multiplyByScalar(transposeMatrix(state.G, state.n, state.m), 18, 1.5*1.5), state.n, state.m, state.m, state.n);
    double* inter1 = transposeMatrix(state.G, state.n, state.m);
    double* inter2 = multiplyByScalar(inter1, state.n*state.m, 1.5*1.5); // TODO make this a variable to tune
    delete[] state.Q;
    state.Q = multiplyMatrices(state.G, inter2, state.n, state.m, state.m, state.n);
    delete[] inter1;
    delete[] inter2;
    state.R = new double[9]{1.5, 0, 0, // TODO hardcoded
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