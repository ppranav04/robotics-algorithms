#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include "ukf.hpp"

static const double kPi = std::acos(-1.0);
static double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

static bool check(const char* name, bool ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << "\n";
    return ok;
}

static Eigen::Vector2d predicted_measurement(const Eigen::Vector3d& x, const Eigen::Matrix3d& P,
                                            const Eigen::Vector2d& u, const Eigen::Vector2d& lm){
    Eigen::Matrix<double, 2, 7> w = ukf::generate_weights(ukf::n, ukf::k, ukf::alpha, ukf::beta);
    Eigen::Matrix<double, 3, 7> prop = ukf::propagate_sigma_points(ukf::generate_sigma_points(x, P), u);
    Eigen::Vector3d pm = ukf::predicted_mean(w, prop);
    Eigen::Matrix3d pc = ukf::predicted_covariance(w, prop, pm);
    Eigen::Matrix<double, 2, 7> obs = ukf::observe_sigma_points(ukf::generate_sigma_points(pm, pc), lm);
    return ukf::mean_observation(w, obs);
}

static bool run_tests() {
    bool all = true;
    const Eigen::Vector2d u0 = Eigen::Vector2d::Zero();
    const Eigen::Matrix3d P = 0.1 * Eigen::Matrix3d::Identity();

    // ---- Test A: circular mean ------------------------------------------
    {
        Eigen::Matrix<double, 2, 7> w = Eigen::Matrix<double, 2, 7>::Zero();
        w(0, 0) = 0.5; w(0, 1) = 0.5;
        Eigen::Matrix<double, 3, 7> pts = Eigen::Matrix<double, 3, 7>::Zero();
        pts(2, 0) = 3.1; pts(2, 1) = -3.1;
        Eigen::Vector3d m = ukf::predicted_mean(w, pts);
        std::cout << "A: theta mean = " << m(2) << "\n";
        all &= check("A: mean of +3.1 and -3.1 is +-pi", std::abs(wrap(m(2) - kPi)) < 1e-9);
    }

    // ---- Test B: control = 0 -> prediction returns the input -------------
    {
        Eigen::Vector3d x(1.0, 2.0, 0.5);
        Eigen::Matrix<double, 2, 7> w = ukf::generate_weights(ukf::n, ukf::k, ukf::alpha, ukf::beta);
        Eigen::Matrix<double, 3, 7> prop = ukf::propagate_sigma_points(ukf::generate_sigma_points(x, P), u0);
        Eigen::Vector3d pm = ukf::predicted_mean(w, prop);
        Eigen::Matrix3d pc = ukf::predicted_covariance(w, prop, pm);
        std::cout << "B: pred mean = " << pm.transpose() << "\nB: pred cov =\n" << pc << "\n";
        all &= check("B1: mean unchanged", (pm - x).norm() < 1e-9);
        all &= check("B2: cov == P + Q*I",
                     (pc - (P + ukf::Q * Eigen::Matrix3d::Identity())).norm() < 1e-9);
    }

    // ---- Test C: z == z-hat -> mean must not move, covariance must shrink --
    {
        Eigen::Vector3d x(1.0, 2.0, 0.5);
        Eigen::Vector2d lm(4.0, 3.0);
        Eigen::Vector2d zhat = predicted_measurement(x, P, u0, lm);
        ukf::UKF r = ukf::ukf_estimation(x, u0, P, zhat, lm);
        Eigen::Matrix3d Ppred = P + ukf::Q * Eigen::Matrix3d::Identity();
        std::cout << "C: est mean = " << r.mean.transpose() << "\nC: est cov =\n" << r.covariance << "\n";
        all &= check("C1: mean does not move", (r.mean - x).norm() < 1e-9);
        all &= check("C2: covariance trace shrinks", r.covariance.trace() < Ppred.trace());
        all &= check("C3: covariance symmetric", (r.covariance - r.covariance.transpose()).norm() < 1e-9);
        all &= check("C4: covariance still positive-definite",
                     Eigen::LLT<Eigen::Matrix3d>(r.covariance).info() == Eigen::Success);
    }

    // ---- Test D: landmark directly behind the robot (bearing at the +-pi seam)
    {
        Eigen::Vector3d x(0.0, 0.0, 0.0);
        Eigen::Vector2d lm(-5.0, 0.0);
        Eigen::Vector2d zhat = predicted_measurement(x, P, u0, lm);
        std::cout << "D: z-hat = " << zhat.transpose() << "\n";
        all &= check("D1: predicted bearing is +-pi (circular mean)",
                     std::abs(std::abs(zhat(1)) - kPi) < 1e-6);

        ukf::UKF r0 = ukf::ukf_estimation(x, u0, P, zhat, lm);
        Eigen::Vector2d z2 = zhat;
        z2(1) += 2.0 * kPi;                       // same angle, different representation
        ukf::UKF r1 = ukf::ukf_estimation(x, u0, P, z2, lm);

        all &= check("D2: z == z-hat -> mean does not move", (r0.mean - x).norm() < 1e-9);
        all &= check("D3: bearing + 2*pi gives the same mean", (r1.mean - r0.mean).norm() < 1e-9);
        all &= check("D4: bearing + 2*pi gives the same covariance",
                     (r1.covariance - r0.covariance).norm() < 1e-9);
    }

    std::cout << (all ? "\nALL TESTS PASS\n" : "\nSOME TESTS FAILED\n");
    return all;
}

static void run_simulation() {
    const int kSteps = 300;   // 30 s at dt = 0.1
    const int kRuns  = 50;

    // One landmark per step, round-robin. ukf_estimation does predict+correct
    // together, so two landmarks in one step would apply the process noise Q twice.
    const std::vector<Eigen::Vector2d> landmarks = {
        Eigen::Vector2d(8.0, 3.0), Eigen::Vector2d(-8.0, 3.0),
        Eigen::Vector2d(0.0, 10.0), Eigen::Vector2d(0.0, -5.0)};

    const Eigen::Vector2d u(1.0, 0.3);   // v [m/s], omega [rad/s] -> a circle of radius ~3.3 m
    const double sq = std::sqrt(ukf::Q); // the truth uses the same noise the filter assumes,
    const double sr = std::sqrt(ukf::R); // so a correct filter must come out consistent
    const Eigen::Matrix3d P0 = Eigen::Vector3d(0.5, 0.5, 0.2).asDiagonal();

    std::vector<double> nees_sum(kSteps, 0.0);
    int failures = 0;
    double final_err_sum = 0.0;

    std::ofstream traj("run0.csv");
    traj << "k,true_x,true_y,true_th,est_x,est_y,est_th,sig_x,sig_y,sig_th,nees\n";

    for (int run = 0; run < kRuns; run++) {
        std::mt19937 rng(1000 + run);                 // fixed seeds -> reproducible runs
        std::normal_distribution<double> N01(0.0, 1.0);
        auto randn3 = [&]() { double a = N01(rng), b = N01(rng), c = N01(rng);
                              return Eigen::Vector3d(a, b, c); };
        auto randn2 = [&]() { double a = N01(rng), b = N01(rng);
                              return Eigen::Vector2d(a, b); };

        Eigen::Vector3d truth = Eigen::Vector3d::Zero();

        ukf::UKF est;
        est.mean = truth + Eigen::LLT<Eigen::Matrix3d>(P0).matrixL() * randn3();  // start off-truth
        est.mean(2) = wrap(est.mean(2));
        est.covariance = P0;

        try {
            for (int k = 0; k < kSteps; k++) {
                // truth: same model the filter assumes (motion model + additive process noise)
                truth = ukf::motion_model(truth, u) + sq * randn3();
                truth(2) = wrap(truth(2));

                // measurement of one landmark, with noise
                const Eigen::Vector2d& lm = landmarks[k % landmarks.size()];
                Eigen::Vector2d z = ukf::observation_model(truth, lm) + sr * randn2();
                z(1) = wrap(z(1));

                est = ukf::ukf_estimation(est.mean, u, est.covariance, z, lm);

                Eigen::Vector3d e = truth - est.mean;
                e(2) = wrap(e(2));
                double nees = e.dot(est.covariance.ldlt().solve(e));   // e^T P^-1 e
                nees_sum[k] += nees;

                if (run == 0) {
                    traj << k << ',' << truth(0) << ',' << truth(1) << ',' << truth(2) << ','
                         << est.mean(0) << ',' << est.mean(1) << ',' << est.mean(2) << ','
                         << std::sqrt(est.covariance(0, 0)) << ',' << std::sqrt(est.covariance(1, 1)) << ','
                         << std::sqrt(est.covariance(2, 2)) << ',' << nees << '\n';
                }
                if (k == kSteps - 1) final_err_sum += (truth.head<2>() - est.mean.head<2>()).norm();
            }
        } catch (const std::runtime_error& ex) {
            failures++;
            std::cout << "run " << run << " failed: " << ex.what() << "\n";
        }
    }

    std::ofstream nf("nees.csv");
    nf << "k,mean_nees\n";
    double total = 0.0; int count = 0;
    for (int k = 0; k < kSteps; k++) {
        double avg = nees_sum[k] / (kRuns - failures);
        nf << k << ',' << avg << '\n';
        if (k >= 20) { total += avg; count++; }
    }

    std::cout << "\nruns failed (covariance not positive-definite): " << failures << " / " << kRuns << "\n"
              << "mean NEES over steps 20+: " << total / count << "   (expected ~3 for a 3-state filter)\n"
              << "mean final position error: " << final_err_sum / (kRuns - failures) << " m\n"
              << "wrote run0.csv and nees.csv\n";
}

int main() {
    if (!run_tests()) {
        std::cout << "\nTests failed, so the simulation was not run.\n";
        return 1;
    }
    run_simulation();
    return 0;
}