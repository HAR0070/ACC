cmake_minimum_required(VERSION 3.5)
project(joystick_control) 

find_package(ament_cmake REQUIRED)

# Install the Python script to the package's lib folder
install(PROGRAMS
  joystick.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
