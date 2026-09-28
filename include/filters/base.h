#ifndef BASE_H
#define BASE_H

// STD
#include <iostream>
#include <mutex>

// Eigen
#include <Eigen/Dense>

namespace filters
{
    class Filter
    {
        public:
            Filter(int, int, const std::string&);
            ~Filter();

            Eigen::VectorXf X_prior;                           // X Prior
            Eigen::VectorXf P_prior;                           // P Prior

            Eigen::VectorXf X_post;                            // X Post
            Eigen::MatrixXf P_post;                            // P Post

            Eigen::MatrixXf K;                                 // Kalman Gain

            virtual void predict() = 0;                         // Predict Step
            virtual void update(const Eigen::VectorXf&) = 0;    // Update Step

            void print_prior();                                 // Print prior
            void print_post();                                  // Print post
            void fuse_kla();                                    // Kullback-Leibler Average
            void fuse_ci();                                     // Covariance Intersection
            void fuse_ici();                                    // Inverse Covariance Intersection

        protected:

            std::mutex  _step_mutex;                            // Prior and post mutex

            unsigned int _dim_x;                                // State dimension - (N)
            unsigned int _dim_z;                                // Measurement dimension - (M)

        private:

            std::string _name;                                  // Name of the filter
    };
};

#endif