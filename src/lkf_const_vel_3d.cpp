#include <filters/lkf.h>

int main(int argc, const char * argv[])
{
    float delta_t = 0.1;

    filters::LKF lkf_const_vel_3d(6, 3, 0, "const_vel_lkf_3d");

    // Setup state transition matrix A
    lkf_const_vel_3d.A <<   1.0, 0.0, 0.0, delta_t, 0.0, 0.0,   // x
                            0.0, 1.0, 0.0, 0.0, delta_t, 0.0,   // y
                            0.0, 0.0, 1.0, 0.0, 0.0, delta_t,   // z
                            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,       // x_dot
                            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,       // y_dot
                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;       // z_dot

    // Setup initial state (X_prior)
    lkf_const_vel_3d.X_prior << 0.0,    // x
                                0.0,    // y
                                0.0,    // z
                                0.0,    // x_dot
                                0.0,    // y_dot
                                0.0;    // z_dot

    // Setup initial Covariance (P_prior)
    lkf_const_vel_3d.P_prior << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   // x
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   // y
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   // z
                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  // x_dot
                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  // y_dot
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01;  // z_dot


    // Setup Process Noise (Q)
    lkf_const_vel_3d.Q <<       0.1, 0.0, 0.0, 0.0, 0.0, 0.0,   // x
                                0.0, 0.1, 0.0, 0.0, 0.0, 0.0,   // y
                                0.0, 0.0, 0.1, 0.0, 0.0, 0.0,   // z
                                0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  // x_dot
                                0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  // y_dot
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.01;  // z_dot

    // Predict
    lkf_const_vel_3d.predict();


    return 0;
}