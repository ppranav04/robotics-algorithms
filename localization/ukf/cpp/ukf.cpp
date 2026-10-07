#include "ukf.hpp"
#include <cmath>
#include <stdexcept>
using namespace std;

// Q noise - process

// R noise - measurement


namespace ukf{
    // motion model
    Eigen::Vector3d motion_model(const Eigen::Vector3d& state, const Eigen::Vector2d& control){
        Eigen::Matrix<double, 3,2> B;
        B << kDt*cos(state(2)), 0,
            kDt*sin(state(2)), 0,
            0                , kDt;

        Eigen::Vector3d state_est = state + B*control;
        return state_est;
    }

    // observation model
    Eigen::Vector2d observation_model(const Eigen::Vector3d& state, const Eigen::Vector2d& landmark_pos){
        double Ix = landmark_pos(0);
        double Iy = landmark_pos(1);
        double x = state(0);
        double y = state(1);
        double theta = state(2);

        double r = sqrt(pow((Ix-x),2) + pow((Iy-y),2));
        double phi = atan2(Iy-y,Ix-x) - theta;
        phi = atan2(sin(phi),cos(phi)); //angle wrapping

        Eigen::Vector2d result;
        result << r,phi;

        return result;
    }

    // sigma poinst generation
    Eigen::Matrix<double, 3, 7> generate_sigma_points(const Eigen::Vector3d& state,
                                                        const Eigen::Matrix3d& covariance){
    
        Eigen::LLT<Eigen::Matrix3d> llt(covariance);

        if (llt.info() != Eigen::Success){
            throw std::runtime_error("sigma_points: covariance is not positive-definite");
        }

        const Eigen::Matrix3d L = llt.matrixL();

        Eigen::Matrix<double, 3, 7> sigma; // n=3 (dimensionality), matrix should be n,2n+1

        sigma.col(0) = state; // column 0
        double gamma = sqrt(n+k);
        for (int i=0; i < 3; i++){
            sigma.col(i+1) = state + gamma*L.col(i); // column 1..3
            sigma.col(i+4) = state - gamma*L.col(i); // column 4..6
        }

        return sigma;
    
    }
    // weights generation
    Eigen::Matrix<double, 2, 7> generate_weights(const int& n, 
                                        const double& k, 
                                        const double& alpha, 
                                        const double& beta){
        Eigen::Matrix<double, 2, 7> weight;

        double lambda = pow(alpha,2)*(n+k) - n;

        weight(0,0) = lambda/(n+lambda);
        weight(1,0) = weight(0,0) + (1 - pow(alpha,2) + beta);

        for (int i=1; i<7; i++){
            weight(0,i) = weight(1,i) = 1/(2*(n+lambda));
        }

        return weight;

    }

    // UKF

    // PREDICTION

    // Sigma points propagation:
    Eigen::Matrix<double, 3, 7> propagate_sigma_points(const Eigen::Matrix<double, 3, 7>& sigmas, const Eigen::Vector2d& controls){
        Eigen::Matrix<double, 3, 7> propagated_sigma_points;

        for (int i=0; i<7; i++){
            propagated_sigma_points.col(i) = ukf::motion_model(sigmas.col(i), controls);
        }

        return propagated_sigma_points;
    }

    // Predicted Mean
    Eigen::Vector3d predicted_mean(const Eigen::Matrix<double, 2, 7>& weights, const Eigen::Matrix<double, 3, 7>& propagated_sigmas){
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();

        double sin_total = 0;
        double cos_total = 0;

        for (int i = 0; i < 7 ; i++){
            mean += weights(0,i) * propagated_sigmas.col(i);

            sin_total += weights(0,i) * sin(propagated_sigmas(2,i));
            cos_total += weights(0,i) * cos(propagated_sigmas(2,i));
        }

        mean(2) = atan2(sin_total,cos_total);
        return mean;
    }

    // Predicted Covariance
    Eigen::Matrix3d predicted_covariance(const Eigen::Matrix<double, 2, 7>& weights, const Eigen::Matrix<double, 3, 7>& propagated_sigmas, const Eigen::Vector3d& pred_mean ){
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();

        for (int i=0; i < 7; i++){
            Eigen::Vector3d residual = propagated_sigmas.col(i) - pred_mean;

            residual(2) = atan2(sin(residual(2)), cos(residual(2)));
            covariance += weights(1, i) * residual * residual.transpose();
        }


        covariance += Q * Eigen::Matrix3d::Identity();       // Q added once, after the sum
        return covariance;
    }
    
    // UKF Algorithm
    UKF ukf_estimation(const Eigen::Vector3d& state, const Eigen::Vector2d& control, const Eigen::Matrix3d& covariance, const Eigen::Vector2d& landmark_pos){
        Eigen::Matrix<double, 2, 7> weights;
        weights = ukf::generate_weights(n, k, alpha, beta); // all know variables 
                
        // Predict
        Eigen::Matrix<double, 3, 7> sigma_points;
        sigma_points = ukf::generate_sigma_points(state, covariance);

        Eigen::Matrix<double, 3, 7> propagated_sigma_points;
        propagated_sigma_points = ukf::propagate_sigma_points(sigma_points, control);

        Eigen::Vector3d Pred_Mean;
        Pred_Mean = ukf::predicted_mean(weights, propagated_sigma_points);

        Eigen::Matrix3d Pred_Cov;
        Pred_Cov = ukf::predicted_covariance(weights, propagated_sigma_points, Pred_Mean);

        UKF ukf_result;
        ukf_result.mean = Pred_Mean;
        ukf_result.covariance = Pred_Cov;

        return ukf_result;
    }

}




