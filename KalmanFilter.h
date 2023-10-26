#ifndef KALMANFILTER_H
#define KALMANFILTER_H

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

#endif
