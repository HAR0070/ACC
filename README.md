# Objective
This repository started to develop lateral and longitudinal control of vehicle. It contains 
- 5 variety of A* algorithms for lateral and longitunidal control  
- Bicycle model based MPC trajectory tracking
- 4 variations of MPC for ACC (python/cpp, 2 different solvers, 2 different formulation)

The A* algorithms were tested in Webots simulator 

ACC algorithms are tested in Webots simulator 
- In P3AT robot 
- In Actual car

## References

This implementation for Hildreth's QP solver and Laguerre based formulation with exponential weights is taken from 
[Model Predictive Control System Design and Implementation Using MATLAB® (2009, Springer)](https://www.researchgate.net/profile/Mohamed-Mourad-Lafifi/post/How_to_design_model_predictive_controller_in_a_factory/attachment/5d2dae0b3843b0b9825ae2b9/AS%3A781245130735617%401563274762980/download/Model+Predictive+Control+System+Design+and+Implementation+Using+MATLAB_Wang.pdf)

## P3AT  testing
[P3AT](https://www.generationrobots.com/media/Pioneer3AT-P3AT-RevA-datasheet.pdf?srsltid=ARcRdnrrxBzPYZ895d-X8wdQuRWk13VdlsN0iitgXQ0ek2n56bAj5MCs) is a 4 wheeler robot where I tested  ACC controller. 
P3AT uses RosAria  
Lidar was used to measure the relative distance, and vehicle odometry to detect speed. 
The full implementation is present in P3AT_testing folder with rosbags. 

## Webots testing
The ACC algorithm was extensively tested in webots for multiple speeds in following scenarios  
- Stop and go 
- Tracking 
- Braking

Results are present in Results folder
ref_vehicle_driver.py is webots reference vehicle controller

MPC controller implementation in webots is present in webots folder 
Webots video [here](https://har0070.github.io/har.github.io/posts/autodrive/#acc)

## Vehicle Implementation 
System identification was performed on the vehicle to derive the state space model for prediction. Along with the controller limits and delay. 

Vehicle's throttle and brake modelling is explained [here](https://har0070.github.io/har.github.io/posts/autonomy/#vehicle-modeling)
Steering modelling [here](https://har0070.github.io/har.github.io/posts/autonomy/#steering-model)
IMU and radar data collection, cleaning and usage [here](https://har0070.github.io/har.github.io/posts/autonomy/#imu-data)

## Complete implementation
Complete implementation code using CANFD (vehicle and steering in can control) with radar based tracking (again can) is present in canfd folder. 
The CAN device drivers are written in C++ from scracth to match vehicle's saftey requirements and complying CANOpen protocol, along with steering and radar, it's made modular and configurable to easily integrate multiple devices. 

## System Info 
P3AT - RosAria, ubuntu 20.04, python 3.8
Webots - R2023b,  C++17, python 3.12
Vehicle - [CANFD](https://robu.in/product/waveshare-industrial-grade-can-can-fd-bus-data-analyzer-usb-to-can-fd-adapter/), Ubuntu 24.04, python 3.12, C++ 17

