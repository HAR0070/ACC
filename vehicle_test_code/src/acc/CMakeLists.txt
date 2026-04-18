cmake_minimum_required(VERSION 3.0.2)
project(acc)

# Use C++17 for structured bindings ( auto [mode, control_action] )
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
)

# Find external math/parsing libraries
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES acc
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

# Tell the compiler where to look for header files (like mpc.hpp and Eigen)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build the main MPC node
add_executable(acc_node src/acc_mpc.cpp)

# Link all required libraries (ROS, YAML, Eigen, OSQP)
target_link_libraries(acc_node
  ${catkin_LIBRARIES}
  yaml-cpp
  Eigen3::Eigen
  osqp
)

# Install the python simulator script so rosrun/roslaunch can find it
catkin_install_python(PROGRAMS
  script/simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)