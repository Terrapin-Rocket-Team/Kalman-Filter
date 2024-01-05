#include "LinearKalmanFilter.h"

KFState initialize(int statevector_size, int measurement_size, int control_size, long* initial_state, long* initial_control){
    KFState state;

    state.F = new long[statevector_size*statevector_size];
    state.G = new long[measurement_size*statevector_size];
    state.X = initial_state;
}