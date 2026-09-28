#include <filters/base.h>

namespace filters
{
    Filter::Filter(int dim_x, int dim_z, const std::string& name) :
        _dim_x(dim_x),
        _dim_z(dim_z),
        _name(name)
    {

        // Prior
        X_prior = Eigen::VectorXf::Zero(_dim_x, 1);
        P_prior = Eigen::MatrixXf::Zero(_dim_x, _dim_x);

        // Post
        X_post = Eigen::VectorXf::Zero(_dim_x, 1);
        P_post = Eigen::MatrixXf::Zero(_dim_x, _dim_x);

        // Kalman Gain
        K = Eigen::VectorXf::Identity(_dim_x, _dim_x);
    }

    Filter::~Filter()
    {

    }

    /**
     * @brief Helper to print the prior states.
     */
    void Filter::print_prior()
    {
        std::lock_guard<std::mutex> lg(_step_mutex);

        std::stringstream ss;
        ss  << "X (Prior): " << std::endl
            << X_prior << std::endl
            << "P (Prior): " << std::endl
            << P_prior << std::endl;
    }

    /**
     * @brief Helper to print the post states.
     */
    void Filter::print_post()
    {
        std::lock_guard<std::mutex> lg(_step_mutex);

        std::stringstream ss;
        ss  << "X (post): " << std::endl
            << X_post << std::endl
            << "P (post): " << std::endl
            << P_post << std::endl;
    }
};