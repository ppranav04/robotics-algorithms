cmake_minimum_required(VERSION 3.16)
project(ekf_landmark_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(ekf ekf.cpp)
target_include_directories(ekf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ekf PUBLIC Eigen3::Eigen)

add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE ekf)