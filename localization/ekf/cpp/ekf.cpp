#include "ekf.hpp"
#include "math.h"
using namespace std;

namespace ekf {
    Eigen::Vector3d motion_model(const Eigen::Vector3d& state,const Eigen::Vector2d& control){

        Eigen::Matrix <double, 3,2> B;
        B << cos(state(2))*kDt, 0,
             sin(state(2))*kDt, 0,
                         0, kDt;

        return state + B*control;
    }
    
    Eigen::Vector2d observation_model(const Eigen::Vector3d& state, const Eigen::Vector2d& measurement){
        double x = state(0);
        double y = state(1);
        double Ix = measurement(0);
        double Iy = measurement(1);

        double r  = sqrt(pow((Ix-x),2) + pow((Iy-y),2));
        double theta = atan2(Iy-y,Ix-x) - state(2);

        Eigen::Vector2d O;
        O << r,
             theta;

        return O;
    }

    Eigen::Matrix3d jacobian_f(const Eigen::Vector3d& state, const Eigen::Vector2d& control){
        Eigen::Matrix3d F;
        F << 1, 0, -control(0)*sin(state(2))*kDt,
                      0, 1, control(0)*cos(state(2))*kDt,
                      0, 0, 1;
        return F;
    }

    Eigen::Matrix <double, 2, 3> jacobian_h(const Eigen::Vector3d& state, const Eigen::Vector2d& measurement){
        double x = state(0);
        double y = state(1);
        double Ix = measurement(0);
        double Iy = measurement(1);

        double r  = sqrt(pow((Ix-x),2) + pow((Iy-y),2));

        Eigen::Matrix <double, 2, 3> H;
        
        H << (x-Ix)/r , (y-Iy)/r, 0,
            (Iy-y)/(r*r), (x-Ix)/(r*r), -1;

        return H;
    }

    EkfResult EKF(const Eigen::Vector3d& xEst, const Eigen::Matrix3d& PEst, const Eigen::Vector2d& u, const Eigen::Vector2d& z, const Eigen::Vector2d& landmark, const Eigen::Matrix3d& Q, const Eigen::Matrix2d& R){
        
        Eigen::Matrix3d jF;
        jF = jacobian_f(xEst,u);

        //Predict 
        Eigen::Vector3d xPred;
        xPred = motion_model(xEst, u);

        Eigen::Matrix3d PPred;
        PPred = jF*PEst*jF.transpose() + Q;

        Eigen::Matrix<double, 2, 3> jH;
        jH = jacobian_h(xPred,landmark);
        
        //Update
        Eigen::Vector2d zPred;
        zPred = observation_model(xPred, landmark);
        Eigen::Vector2d Y;
        Y = z - zPred;
        Y(1) = atan2(sin(Y(1)),cos(Y(1)));
        Eigen::Matrix2d S;
        S = jH*PPred*jH.transpose() + R;
        Eigen::Matrix<double, 3, 2> K;
        K = PPred*jH.transpose()*S.inverse();

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

        EkfResult Result;
        Result.xEst = xPred + K*Y;
        Result.PEst = (I - K*jH)*PPred;

        return Result;

    }
}  // namespace ekf