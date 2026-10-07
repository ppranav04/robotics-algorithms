#pragma once
#include <Eigen/Dense>

namespace ukf {

    constexpr double kDt = 0.1;
    Eigen::Vector3d motion_model(const Eigen::Vector3d& state, 
                                const Eigen::Vector2d& control);
    Eigen::Vector2d observation_model(const Eigen::Vector3d& state, 
                                    const Eigen::Vector2d& landmark_pos);
    Eigen::Matrix<double, 3, 7> sigma_points(const Eigen::Vector3d& state, 
                                            const Eigen::Matrix3d& covariance,
                                            const double& gamma);
    Eigen::Matrix<double, 2, 7> weights(const int& n, const double& k, const double& alpha, const double& beta);
    
    struct Predict_Result {
        Eigen::Vector3d mean;
        Eigen::Matrix3d covariance;
    };

    Eigen::Vector3d mean(const Eigen::Vector3d& state_estimate,  
                             const Eigen::Matrix<double, 2, 7>& weights);
    Eigen::Matrix3d covariance(const Eigen::Vector3d& state_estimate,
                                const Eigen::Vector3d& predicted_mean,
                                const Eigen::Matrix<double, 2, 7>& weights,
                                const double& R);
}