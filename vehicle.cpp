
#include "ros/ros.h"
#include "std_msgs/Float32.h"  // For throttle and brake
#include "std_msgs/Float32MultiArray.h"  // To publish current states 
// #include "radar_msgs"  // For radar data  
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>


class pid_controller {
public:

    pid_controller(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integral_error_(0.0f), prev_error_(0.0f) {

    }

    float compute_control(float error, float dt) {
        integral_error_ += error * dt;
        if (integral_error_ > 10.0f) integral_error_ = 10.0f; // Anti-windup
        if (integral_error_ < -10.0f) integral_error_ = -10.0f;
        
        if (error > -0.5f && error < 0.5f) {
            integral_error_ = 0.0f; // Reset integral error when close to setpoint
        }

        float derivative_error = (error - prev_error_) / dt;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;
    }
}

class states {
    public: 
    states(float dr, float vr, float ah ,  float u_prev) : dr(dr), vr(vr), ah_(ah), u_prev_(u_prev) {

    }

    void radar_callback(const radar_msgs::RadarData& msg) {
        // Extract relevant data from the radar message
        dr = msg.distance_to_lead_vehicle;  // Distance to lead vehicle
        vr = msg.relative_velocity;         // Relative velocity to lead vehicle
    }

    void imu_callback(const sensor_msgs::Imu& msg) {  // fixaxis -- change 
        // Extract relevant data from the IMU message
        ah_ = msg.linear_acceleration.x;  // Longitudinal acceleration
        vh_ = msg.linear.velocity.x;  // Longitudinal velocity
    }

    float get_state() const {
        return dr , vr;
    }
}


int main(int argc, char **argv) {

    try {
        // Load the YAML file into a YAML::Node object
        YAML::Node config = YAML::LoadFile("config.yaml");

        // Access nested values using chained bracket notation
        const float kp = config["pid"]["kp"].as<float>();
        const float kd = config["pid"]["kd"].as<float>();
        const float ki = config["pid"]["ki"].as<float>();
        const float thw = config["pid"]["thw"].as<float>();
        const float d0 = config["pid"]["d0"].as<float>();

    } catch (const std::exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return 1;
    }

    pid_controller pid(kp, ki, kd);
    states current_states(0.0f, 0.0f, 0.0f, 0.0f); // Initialize states with default values

    ros::init(argc, argv, "acc_controller");
    ros::NodeHandle n;

    ros::Publisher state_pub = n.advertise<std_msgs::Float32MultiArray>("/states", 1000);
    ros::Publisher throttle_pub = n.advertise<std_msgs::Float32>("/throttle", 1000);
    ros::Publisher brake_pub = n.advertise<std_msgs::Float32>("/brake", 1000);

    ros::Subscriber radar_sub = n.subscribe("/radar_data", 1000, current_states.radar_callback); // Subscribe to radar data topic
    ros::Subscriber imu_sub = n.subscribe("/imu_data", 1000, current_states.imu_callback); // Subscribe to imu data topic
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // Compute control action using PID controller
        dr , vr = current_states.get_state();

        float error = dr - (vr * thw + d0);  // Desired distance - actual distance
        // try with velocity error for  Kd
        float control_action = pid.compute_control(error, 0.1f);  // Assuming dt = 0.1s

        // Publish throttle and brake commands based on control action
        std_msgs::Float32 throttle_msg;
        std_msgs::Float32 brake_msg;

        if (control_action > 0) {
            throttle_msg.data = std::min(control_action, 1.0f);  // Limit throttle to [0, 1]
            brake_msg.data = 0.0f;
        } else {
            throttle_msg.data = 0.0f;
            brake_msg.data = std::min(-control_action, 1.0f);  // Limit brake to [0, 1]
        }

        throttle_pub.publish(throttle_msg);
        brake_pub.publish(brake_msg);

    }
    return 0;

}
chatter