#include "LinearKalmanFilter.h"

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control){
    KFState state;

    state.F = new double[statevector_size*statevector_size];
    state.G = new double[measurement_size*statevector_size];
    state.X = initial_state;
    state.U = initial_control;
}