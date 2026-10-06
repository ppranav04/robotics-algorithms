#include <iostream>
#include "ukf.hpp"
using namespace std;

int main(){
    Eigen::Vector3d eg_state;
    eg_state << 1,1,0.78;

    Eigen::Vector2d eg_control;
    eg_control << 5,0.52;

    Eigen::Vector2d eg_landmark;
    eg_landmark << 1,1;

    Eigen::Vector3d motion_model_result;
    motion_model_result = ukf::motion_model(eg_state, eg_control);

    cout << motion_model_result << '\n';

    Eigen::Vector2d obs_model_result;
    obs_model_result = ukf::observation_model(eg_state, eg_landmark);

    cout << obs_model_result << '\n';

    int n= 3;
    double a = 1;
    double k = 0;
    double b = 2;

    Eigen::Matrix<double, 2, 7> weights_result;

    weights_result = ukf::weights(n,k,a,b);
    cout << weights_result << '\n';
}