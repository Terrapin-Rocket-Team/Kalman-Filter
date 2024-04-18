#include "../../MatrixMult/Matrix.h"
#include "Config.h"

#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H

typedef struct {
    Matrix F; // State Transition Matrix
    Matrix G; // Control Matrix
    Matrix H; // Observation Matrix
    Matrix P; // Estimate Covariance Matrix
    Matrix R; // Measurement Uncertainty Matrix
    Matrix K; // Kalman Gain Matrix
    Matrix Q; // Process Noise Matrix
    Matrix U; // Control Vector
    Matrix X; // State Vector
} KFState;

class LinearKalmanFilter{
    public: 
        LinearKalmanFilter(Config c);
        void predict_state();
        void estimate_state(Matrix measurement);
        void calculate_kalman_gain();
        void covariance_update();
        void covariance_extrapolate();
        void calculate_initial_values();
        Matrix iterate(Config c);
    
    private:
        KFState state;
};
#endif