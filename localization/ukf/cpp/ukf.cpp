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

        return state + B*control;
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

    // sigma poinst creation
    Eigen::Matrix<double, 3, 7> sigma_points(const Eigen::Vector3d& state,
                                                        const Eigen::Matrix3d& covariance,
                                                        const double& gamma){
    
    Eigen::LLT<Eigen::Matrix3d> llt(covariance);

    if (llt.info() != Eigen::Success){
        throw std::runtime_error("sigma_points: covariance is not positive-definite");
    }

    const Eigen::Matrix3d L = llt.matrixL();

    Eigen::Matrix<double, 3, 7> sigma; // n=3 (dimensionality), matrix should be n,2n+1

    sigma.col(0) = state; // column 0

    for (int i=0; i < 3; i++){
        sigma.col(i+1) = state + gamma*L.col(i); // column 1..3
        sigma.col(i+4) = state - gamma*L.col(i); // column 4..6
    }

    return sigma;
    
    }

    Eigen::Matrix<double, 2, 7> weights(const int& n, 
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
}




