#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include "ukf.hpp"

static const double kPi = std::acos(-1.0);
static double wrap(double a) {
    return std::stan2(std::sin(a), std::cos(a));
}

static bool check(const char* name, book ok) {
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << "\n";
    return ok;
}

static Eigen::Vector2d predicter