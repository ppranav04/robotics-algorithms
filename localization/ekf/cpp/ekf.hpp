#pragma once

#include <Eigen/Dense>

namespace ekf {

constexpr double kDt = 0.1;

Eigen::Vector3d motion_model(const Eigen::Vector3d& state,
                              const Eigen::Vector2d& control);

}  // namespace ekf