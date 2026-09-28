#include <filters/lkf.h>

namespace filters
{
    LKF::LKF(int dim_x, int dim_z, int dim_u, const std::string& name) : Filter(dim_x, dim_z, name),
                                                                    _dim_u(dim_u)
    {
        H = Eigen::MatrixXf::Identity(dim_z, dim_x);    // (M * N)
        A = Eigen::MatrixXf::Identity(dim_x, dim_x);    // (N * N)
        B = Eigen::MatrixXf::Identity(dim_x, _dim_u);   // (N * K)
        u = Eigen::VectorXf::Zero(_dim_u, 1);           // (K * 1)
        Q = Eigen::MatrixXf::Zero(dim_x, dim_x);        // (N * N)
        R = Eigen::MatrixXf::Zero(dim_z, dim_z);        // (M * M)
        I = Eigen::MatrixXf::Identity(dim_x, dim_x);    // (N * N)
        S = Eigen::MatrixXf::Identity(dim_z, dim_z);    // (M * M)
    }

    LKF::~LKF()
    {

    }

    /**
     * @brief Kalman Filter predict step.
     * 
     * x- = Ax+ + Bu        // Predict State
     * P- = AP+(A^T) + Q    // Predict Covariance
     */
    void LKF::predict()
    {
        std::lock_guard<std::mutex> lg(_step_mutex);

        X_prior = A * X_post + B * u;
        P_prior = A * P_post * A.transpose() + Q;
    }

    /**
     * @brief Kalman Filter Update step.
     * 
     * y = z - Hx-                      // Innovation
     * S = HP(H^T) + R                  // Innovation Covariance
     * K = P(H^T)(S^-1)                 // Kalman Gain
     * x+ = x- + Ky                     // Updated State
     * P+ = (I - KH)P-                  // Updated Covariance
     * P+ = (I-KH)P-(I-KH)^T + KR(K^T)  // Updated Covariance Joseph Form
     * 
     * @param z - Measurement.
     */
    void LKF::update(const Eigen::VectorXf& z)
    {
        y = z - H * X_prior;
        S = H * P_prior * H.transpose() + R;
        K = P_prior * H.transpose() * S.inverse();
        X_post = X_prior + K * y;
        Eigen::MatrixXf T = (I - K * H);
        P_post = T * P_prior * T.transpose() + K * R * K.transpose();
    }
};