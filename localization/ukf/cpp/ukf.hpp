#pragma once
#include <Eigen/Dense>

namespace ukf {

    constexpr double kDt = 0.1;
    constexpr double R = 0.01;
    constexpr double Q = 0.01;
    constexpr int n = 3; // dimenstionality
    constexpr int col = 2*n+1;
    constexpr double k = 0;
    constexpr double alpha = 1;
    constexpr double beta = 2;


    Eigen::Vector3d motion_model(const Eigen::Vector3d& state, 
                                const Eigen::Vector2d& control);
    Eigen::Vector2d observation_model(const Eigen::Vector3d& state, 
                                    const Eigen::Vector2d& landmark_pos);
    Eigen::Matrix<double, 3, 7> generate_sigma_points(const Eigen::Vector3d& state, 
                                            const Eigen::Matrix3d& covariance);
    Eigen::Matrix<double, 2, 7> generate_weights(const int& n, const double& k, const double& alpha, const double& beta);
    
    struct UKF {
        Eigen::Vector3d mean;
        Eigen::Matrix3d covariance;
    };

    // Prediction
  
    Eigen::Matrix<double, 3, 7> propagate_sigma_points(const Eigen::Matrix<double, 3, 7>& sigmas, 
                                                        const Eigen::Vector2d& controls);
    Eigen::Vector3d predicted_mean(const Eigen::Matrix<double, 2, 7>& weights, 
                                    const Eigen::Matrix<double, 3, 7>& propagated_sigmas);
    Eigen::Matrix3d predicted_covariance(const Eigen::Matrix<double, 2, 7>& weights, 
                                    const Eigen::Matrix<double, 3, 7>& propagated_sigmas,
                                    const Eigen::Vector3d& pred_mean);

    // Correction
    Eigen::Matrix<double, 2, 7> observe_sigma_points(const Eigen::Matrix<double, 3, 7>& sigmas,
                                        const Eigen::Vector2d& landmark_pos); //
    Eigen::Vector2d mean_observation(const Eigen::Matrix<double, 2, 7>& weights, 
                                        const Eigen::Matrix<double, 2, 7>& observed_sigma_points); // Innovation
    Eigen::Matrix2d innovation_covariance(const Eigen::Matrix<double, 2, 7>& weights, 
                                            const Eigen::Matrix<double, 2, 7>& observed_sigma_points, 
                                            const Eigen::Vector2d& mean_observation); // Innovation Covariance

    Eigen::Matrix<double, 3, 2> cross_covariance(const Eigen::Matrix<double, 2, 7>& weights,
                                                    const Eigen::Matrix<double,3, 7>& propogated_sigma_points,
                                                    const Eigen::Vector3d& predMean,
                                                    const Eigen::Matrix<double, 2, 7>& observed_sigma_points,
                                                    const Eigen::Vector2d& obsMean);
        
    UKF ukf_estimation(const Eigen::Vector3d& state, 
                        const Eigen::Vector2d& control,
                        const Eigen::Matrix3d& covariance,
                        const Eigen::Vector2d& z,
                        const Eigen::Vector2d& landmark_pos);

    //



    


}