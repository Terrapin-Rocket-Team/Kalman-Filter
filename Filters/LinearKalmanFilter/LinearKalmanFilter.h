#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H

typedef struct {
    long** F; // State Transition Matrix
    long** G; // Control Matrix
    long** H; // Control Vector
    long** P; // State Vector
    long** R; // Measurement Uncertainty Matrix
    long** K; // Kalman Gain Matrix
    long** Q; // Process Noise Matrix
    long* U;  // Control Vector
    long* X;  // State Vector

} KFState;

void predict_state(KFState *state){

}

void estimate_state(KFState *state, int *measurement){

}

void calculate_kalman_gain(KFState *state){

}

void covariance_update(KFState *state){
    
}

#endif
