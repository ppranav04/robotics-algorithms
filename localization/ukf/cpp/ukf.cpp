#include "ukf.hpp"
#include <cmath>
using namespace std;

// Q noise - process

// R noise - measurement

// motion model
namespace ukf{
    Eigen::Vector3d motion_model(const Eigen::Vector3d& state, const Eigen::Vector2d& control){
        Eigen::Matrix<double, 3,2> B;
        B << kDt*cos(state(2)), 0,
            kDt*sin(state(2)), 0,
            0                , kDt;

        return state + B*control;
    }

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
}
// observation model

// sigma poinst creation

