cmake_minimum_required(VERSION 3.5)
project(test1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -w disables warnings (from your snippet), though -Wall is usually safer
set(CMAKE_CXX_FLAGS "-g -w -pthread")

# 3. Include the current directory so it finds "controlcanfd.h"
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test1 main.cpp)
find_package(glog CONFIG REQUIRED)


target_link_libraries(test1
    pthread
    dl
    glog
    yaml-cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/libcontrolcanfd.so
)