cmake_minimum_required(VERSION 3.16)
project(ukf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


add_library(ukf ukf.cpp)
target_link_libraries(ukf PUBLIC Eigen3::Eigen)

target_include_directories(ukf PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(demo demo.cpp)
target_link_libraries(demo PRIVATE ukf)