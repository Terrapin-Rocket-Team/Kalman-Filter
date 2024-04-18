#include "Config.h"
#include "../../MatrixMult/Matrix.h"

Config initialize_filter(double dt){
    Matrix X(6, 1, new double[6] {0, 0, 0, 0, 0, 0});
    Matrix U(3, 1, new double[3] {0, 0, 0});
    Matrix P(6, 6, new double[36]{100, 0, 0, 100, 0, 0,
                            0, 100, 0, 0, 100, 0,
                            0, 0, 100, 0, 0, 100,
                            100, 0, 0, 100, 0, 0,
                            0, 100, 0, 0, 100, 0,
                            0, 0, 100, 0, 0, 100});
    Matrix F_mat(6, 6, new double[36]{1, 0, 0, dt, 0, 0,
                            0, 1, 0, 0, dt, 0,
                            0, 0, 1, 0, 0, dt,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1});
    Matrix G(6, 3, new double[18]{0.5*dt*dt, 0, 0,
                            0, 0.5*dt*dt, 0,
                            0, 0, 0.5*dt*dt,
                            dt, 0, 0,
                            0, dt, 0,
                            0, 0, dt});
    Matrix R(3, 3, new double[9]{0.5, 0, 0,
                            0, 0.5, 0,
                            0, 0, 0.5});
    return Config{
        X, U, P, F_mat, G, R, Matrix()
    };
 
}

Config iterate_filter(double dt, double* meas_arr, double* control_arr, int num_satellites){
    bool nanGPS = false;
    for (int i = 0; i < 3; i++){
      if (isnan(meas_arr[i])){
        meas_arr[i] = 0.0;
        nanGPS = true;
      }
    }
    Matrix X(3, 1, meas_arr);
    for (int i = 0; i < 3; i++){
      if (isnan(control_arr[i])){
        control_arr[i] = 0.0;
      }
    }
    Matrix U(3, 1, control_arr);
    Matrix F_mat(6, 6, new double[36]{1, 0, 0, dt, 0, 0,
                              0, 1, 0, 0, dt, 0,
                              0, 0, 1, 0, 0, dt,
                              0, 0, 0, 1, 0, 0,
                              0, 0, 0, 0, 1, 0,
                              0, 0, 0, 0, 0, 1});
    Matrix G(6, 3, new double[18]{0.5*dt*dt, 0, 0,
                              0, 0.5*dt*dt, 0,
                              0, 0, 0.5*dt*dt,
                              dt, 0, 0,
                              0, dt, 0,
                              0, 0, dt});

    Matrix H;

    if (num_satellites > 5 && !nanGPS){
      H = Matrix(3, 6, new double[18]{1, 0, 0, 0, 0, 0,
                              0, 1, 0, 0, 0, 0,
                              0, 0, 1, 0, 0, 0});
    }
    else {
      H = Matrix(3, 6, new double[18]{0, 0, 0, 0, 0, 0,
                              0, 0, 0, 0, 0, 0,
                              0, 0, 1, 0, 0, 0});
    }

    return Config{
        X, U, Matrix(), F_mat, G, Matrix(), H
    };
}
