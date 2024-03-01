#include <ArduinoEigen.h>

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Vector;
using Eigen::VectorXd;

#ifndef LINEARKALMANFILTER_H
#define LINEARKALMANFILTER_H


struct KFState {
    Vector<double, 6> X; // State vector
    Vector<double, 3> U; // Control vector
    Matrix<double, 6, 6> P; // State covariance matrix
    Matrix<double, 6, 3> K; // Kalman gain matrix
    Matrix<double, 6, 6> Q; // Process noise covariance matrix
    Matrix<double, 3, 3> R; // Measurement noise covariance matrix
    Matrix<double, 6, 6> F; // State transition matrix
    Matrix<double, 6, 3> G; // Control input matrix
    Matrix<double, 3, 6> H; // Measurement matrix
};

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control);
KFState predict_state(KFState state);
KFState estimate_state(KFState state, double *measurement);
KFState calculate_kalman_gain(KFState state);
KFState covariance_update(KFState state);
KFState covariance_extrapolate(KFState state);
KFState calculate_initial_values(KFState state, float dt);
KFState iterate(KFState state, float dt, double* measurement_vector, double* control_vector, bool has_gps);
#endif