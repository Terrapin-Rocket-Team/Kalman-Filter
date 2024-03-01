#include "LinearKalmanFilter.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Map;

KFState initialize(int statevector_size, int measurement_size, int control_size, double* initial_state, double* initial_control){
    KFState state;
    state.X = Map<VectorXd>(initial_state, statevector_size);
    state.U = Map<VectorXd>(initial_control, control_size);
    Serial.print("U rows: ");
    Serial.println(state.U.rows());
    Serial.print("U cols: ");     
    Serial.println(state.U.cols());
    state.P = MatrixXd::Identity(statevector_size, statevector_size) * 100;
    state.Q = MatrixXd::Zero(statevector_size, statevector_size); // Adjust as needed
    state = calculate_initial_values(state, 0.01);
    return state;
}
KFState predict_state(KFState state){
    VectorXd inter1 = state.F * state.X;
    VectorXd inter2 = state.G * state.U;
    state.X = inter1 + inter2;
    return state;
}
KFState estimate_state(KFState state, double *measurement, int measurement_size){
    VectorXd measurement_vec = Map<VectorXd>(measurement, measurement_size);
    VectorXd inter1 = measurement_vec - state.H*state.X;
    state.X = state.X + state.K*inter1;
    return state;
}
KFState calculate_kalman_gain(KFState state){
    //K = obj.P * obj.H' * inv(obj.H * obj.P * obj.H' + obj.R);
    MatrixXd inter1 = state.H.transpose();
    MatrixXd inter2 = (state.H * state.P * inter1) + state.R;
    MatrixXd inter3 = state.H.transpose();
    state.K = state.P * inter3 * inter2.inverse();
    return state;
}
KFState covariance_update(KFState state){
    //obj.P = (eye(n) - obj.K*obj.H)*obj.P*(eye(n) - obj.K*obj.H)' + obj.K*obj.R*obj.K';
    state.P = (MatrixXd::Identity(6, 6) - state.K*state.H)*state.P*(MatrixXd::Identity(6, 6) - 
        state.K*state.H).transpose() + state.K*state.R*state.K.transpose();
    return state;
}
KFState covariance_extrapolate(KFState state){
    //obj.P = obj.F*obj.P*obj.F'+ obj.Q;
    state.P = state.F*state.P*state.F.transpose() + state.Q;
    return state;
}
KFState calculate_initial_values(KFState state, float dt){
    state.F << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    state.G << 0.5*dt*dt, 0, 0,
            0, 0.5*dt*dt, 0,
            0, 0, 0.5*dt*dt,
            dt, 0, 0,
            0, dt, 0,
            0, 0, dt;


    state.Q = (state.G*(state.G.transpose()))*1.44;
    state = predict_state(state);
    state = covariance_extrapolate(state);
    return state;
}
KFState iterate(KFState state, float dt, double* measurement_vector, double* control_vector, bool has_gps){
    state.F << 1, 0, 0, dt, 0, 0,
            0, 1, 0, 0, dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;

    state.G << 0.5*dt*dt, 0, 0,
            0, 0.5*dt*dt, 0,
            0, 0, 0.5*dt*dt,
            dt, 0, 0,
            0, dt, 0,
            0, 0, dt;

    if (has_gps) {
        state.H << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0;
    } else {
        state.H << 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0;
    }

    state.R << 0.5, 0, 0,
            0, 0.5, 0,
            0, 0, 0.5;

    state.Q = (state.G*(state.G.transpose()))*1.44;

    state = calculate_kalman_gain(state);
    state = estimate_state(state, measurement_vector, 3);
    state = covariance_update(state);
    state = predict_state(state);
    state = covariance_extrapolate(state);
    return state;
}