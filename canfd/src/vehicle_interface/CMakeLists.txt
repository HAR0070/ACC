cmake_minimum_required(VERSION 3.5)
project(vehicle_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -w disables warnings (from your snippet), though -Wall is usually safer
set(CMAKE_CXX_FLAGS "-g -w -pthread")

# 3. Include the current directory so it finds "controlcanfd.h"
include_directories(
   ../../include
    )

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vehicle_interface_node vehicle_interface.cpp)

ament_target_dependencies(vehicle_interface_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  ament_index_cpp
)

target_link_libraries(vehicle_interface_node
    pthread
    dl
    glog::glog
    yaml-cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/libcontrolcanfd.so
)

install(TARGETS vehicle_interface_node
    DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY config
	DESTINATION share/${PROJECT_NAME})

# Install the library so the node finds it at runtime
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../include/libcontrolcanfd.so
    DESTINATION lib/${PROJECT_NAME} )

ament_package()
