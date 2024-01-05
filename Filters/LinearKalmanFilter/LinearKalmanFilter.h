#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H

typedef struct {
    long* F; // State Transition Matrix
    long* G; // Control Matrix
    long* H; // Observation Matrix
    long* P; // Estimate Covariance Matrix
    long* R; // Measurement Uncertainty Matrix
    long* K; // Kalman Gain Matrix
    long* Q; // Process Noise Matrix
    long* U; // Control Vector
    long* X; // State Vector

} KFState;

KFState initialize(int statevector_size, int measurement_size, int control_size, long* initial_state, long* initial_control){
 
}
void predict_state(KFState *state){

}

void estimate_state(KFState *state, int *measurement){

}

void calculate_kalman_gain(KFState *state){

}

void covariance_update(KFState *state){
    
}

long* iterate(KFState *state, float dt, long* measurement_vector, long* control_vector){

}

#endif
