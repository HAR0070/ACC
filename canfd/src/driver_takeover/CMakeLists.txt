cmake_minimum_required(VERSION 3.5)
project(driver_takeover)

find_package(ament_cmake REQUIRED)

install( PROGRAMS
    driver_takeover.py 
    DESTINATION lib/${PROJECT_NAME}    
)

install (DIRECTORY model
    DESTINATION lib/${PROJECT_NAME})

ament_package()