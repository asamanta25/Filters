#ifndef LKF_H
#define LKF_H

#include <filters/base.h>

namespace filters
{
    class LKF: public Filter
    {
        public:

            LKF(int, int, int, const std::string&);
            ~LKF();

            Eigen::MatrixXf H;                      // Measurement Matrix (M X N)
                                                    // Public because the user can easily access
            Eigen::MatrixXf A;                      // State transition matrix
            Eigen::MatrixXf B;                      // Control input step
            Eigen::MatrixXf Q;                      // Process Noise
            Eigen::MatrixXf S;                      // Innovation Covariance
            Eigen::MatrixXf R;                      // Measurement Noise
            Eigen::MatrixXf I;                      // Identity
            Eigen::VectorXf u;                      // Control Input
            Eigen::VectorXf y;                      // Innovation

            void predict();                         // Predict Step
            void update(const Eigen::VectorXf&);    // Update Step

        private:

            unsigned int _dim_u;                    // Control Input Dimenstions
    };
};

#endif