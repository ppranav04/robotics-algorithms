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

    Eigen ::Vector2d obs_model_result;
    obs_model_result = ukf::observation_model(eg_state, eg_landmark);

    cout << obs_model_result << '\n';
}