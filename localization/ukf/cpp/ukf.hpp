#pragma once
#include <Eigen/Dense>

namespace ukf {

    constexpr double kDt = 0.1;
    Eigen::Vector3d motion_model(const Eigen::Vector3d& state, 
                                const Eigen::Vector2d& control);
    Eigen::Vector2d observation_model(const Eigen::Vector3d& state, const Eigen::Vector2d& landmark_pos);
    
}