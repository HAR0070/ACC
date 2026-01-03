cmake_minimum_required(VERSION 3.0)
project(MPC_Project)

# Tell CMake to look in the current Conda environment
set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(mpc_node mpc.cpp)

# Link the libraries
target_link_libraries(mpc_node Eigen3::Eigen yaml-cpp)