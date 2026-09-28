#include <iostream>
#include "ekf.hpp"

int main() {
    std::cout << "build works\n";
    Eigen::Vector3d s(0, 0, 0);
    Eigen::Vector2d c(1, 0);
    std::cout << ekf::motion_model(s, c).transpose() << "\n";
    return 0;
}