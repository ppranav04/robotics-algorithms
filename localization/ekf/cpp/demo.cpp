#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <vector>

#include "ekf.hpp"

namespace {

constexpr double kSensorRange = 8.0;
constexpr double kSimTime = 80.0;

const std::vector<Eigen::Vector2d> kLandmarks = {
    Eigen::Vector2d(5.0, 5.0),
    Eigen::Vector2d(-3.0, 6.0),
    Eigen::Vector2d(4.0, -4.0),
    Eigen::Vector2d(-5.0, -3.0),
};

Eigen::Vector2d calc_input() {
    return Eigen::Vector2d(1.0, 0.15);
}


std::optional<Eigen::Vector2d> nearest_visible_landmark(
        const Eigen::Vector3d& x_true) {
    std::optional<Eigen::Vector2d> best;
    double best_dist = 0.0;
    for (const auto& lm : kLandmarks) {
        double d = std::hypot(lm(0) - x_true(0), lm(1) - x_true(1));
        if (d <= kSensorRange && (!best.has_value() || d < best_dist)) {
            best = lm;
            best_dist = d;
        }
    }
    return best;
}

}  

int main() {
   
    Eigen::Matrix3d Q = Eigen::Vector3d(0.05, 0.05, 1.0 * M_PI / 180.0)
                             .cwiseAbs2().asDiagonal();
    Eigen::Matrix2d R = Eigen::Vector2d(0.3, 3.0 * M_PI / 180.0)
                             .cwiseAbs2().asDiagonal();
    Eigen::Matrix2d input_noise_var =
        Eigen::Vector2d(0.3, 5.0 * M_PI / 180.0).cwiseAbs2().asDiagonal();
    Eigen::Matrix2d input_noise_std = input_noise_var.cwiseSqrt();
    Eigen::Matrix2d meas_noise_std = R.cwiseSqrt();

    std::mt19937 rng(0);  // fixed seed, matches Python's seed=0
    std::normal_distribution<double> gauss(0.0, 1.0);
    auto draw2 = [&]() { return Eigen::Vector2d(gauss(rng), gauss(rng)); };

    Eigen::Vector3d x_true = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_dr = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_est = Eigen::Vector3d::Zero();
    Eigen::Matrix3d p_est = Eigen::Matrix3d::Identity() * 0.1;

    std::ofstream csv("ekf_cpp_run.csv");
    csv << "t,x_true,y_true,th_true,x_dr,y_dr,th_dr,"
           "x_est,y_est,th_est,p00,p11,landmark\n";

    double t = 0.0;
    while (t <= kSimTime) {
        t += ekf::kDt;
        Eigen::Vector2d u = calc_input();

      
        x_true = ekf::motion_model(x_true, u);

      
        Eigen::Vector2d u_noisy = u + input_noise_std * draw2();
        x_dr = ekf::motion_model(x_dr, u_noisy);

       
        auto landmark = nearest_visible_landmark(x_true);
        int landmark_idx = -1;

        if (!landmark.has_value()) {
           
            Eigen::Matrix3d jF = ekf::jacobian_f(x_est, u_noisy);
            x_est = ekf::motion_model(x_est, u_noisy);
            p_est = jF * p_est * jF.transpose() + Q;
        } else {
            for (size_t i = 0; i < kLandmarks.size(); ++i) {
                if ((kLandmarks[i] - *landmark).norm() < 1e-9) {
                    landmark_idx = static_cast<int>(i);
                    break;
                }
            }

            Eigen::Vector2d z_true = ekf::observation_model(x_true, *landmark);
            Eigen::Vector2d z = z_true + meas_noise_std * draw2();
            z(1) = std::atan2(std::sin(z(1)), std::cos(z(1)));  // wrap

            ekf::EkfResult result =
                ekf::EKF(x_est, p_est, u_noisy, z, *landmark, Q, R);
            x_est = result.xEst;
            p_est = result.PEst;
        }

        csv << t << ',' << x_true(0) << ',' << x_true(1) << ',' << x_true(2)
            << ',' << x_dr(0) << ',' << x_dr(1) << ',' << x_dr(2) << ','
            << x_est(0) << ',' << x_est(1) << ',' << x_est(2) << ','
            << p_est(0, 0) << ',' << p_est(1, 1) << ',' << landmark_idx
            << '\n';
    }

    std::cout << "Wrote ekf_cpp_run.csv (" << t / ekf::kDt << " steps)\n";
    return 0;
}