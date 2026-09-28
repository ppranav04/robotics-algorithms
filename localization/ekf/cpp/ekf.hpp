#pragma once
#include <Eigen/Dense>

namespace ekf {

constexpr double kDt = 0.1;

Eigen::Vector3d motion_model(const Eigen::Vector3d& state,
                              const Eigen::Vector2d& control);
Eigen::Vector2d observation_model(const Eigen::Vector3d& state, const Eigen::Vector2d& measurement);
Eigen::Matrix3d jacobian_f(const Eigen::Vector3d& state, const Eigen::Vector2d& control);
Eigen::Matrix <double, 2, 3> jacobian_h(const Eigen::Vector3d& state, const Eigen::Vector2d& measurement);
struct EkfResult {
    Eigen::Vector3d xEst;
    Eigen::Matrix3d PEst;
};
EkfResult EKF(const Eigen::Vector3d& xEst, const Eigen::Matrix3d& PEst,const Eigen::Vector2d& u, const Eigen::Vector2d& z, const Eigen::Vector2d& landmark, const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R);


}  // namespace ekf