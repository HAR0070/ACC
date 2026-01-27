#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <glog/logging.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "vehicle_can.hpp"
#include "steering_can.hpp"

using namespace std::chrono_literals;

// Steering and throttle command come as twist
// both feedback go as float 32 array

class vehicle_interface : public rclcpp::Node
{

public:
    vehicle_interface(steering_can * str , vehicle_can* veh) : steering(str) ,
                                                                vehicle(veh),
                                                                Node("vehicle_interface"),
                                                                count_(0){

        // Inside vehicle_interface constructor
        callback_group_subscribers_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscribers_;

        pub_str_fb = this->create_publisher<std_msgs::msg::Float32MultiArray>("/steering_feedback" , 10);
        pub_vehicle_fb = this->create_publisher<std_msgs::msg::Float32MultiArray>("/vehicle_feedback" , 10);

        sub_str_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
                "/steering_pub" , 10 , std::bind(&vehicle_interface::steering_callback , this , std::placeholders::_1) , sub_opt);

        sub_veh_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
            "/vehicle_throttle" , 10 , std::bind(&vehicle_interface::throttle_callback, this , std::placeholders::_1), sub_opt);

        timer_ = this->create_wall_timer(
            20ms, 
            std::bind(&vehicle_interface::timer_callback, this), 
            callback_group_timer_); // Pass group  // full loop runs in 50Hz

    }

private:
    steering_can* steering;
    vehicle_can* vehicle;

    rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_str_cmd;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_veh_cmd;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_str_fb;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_vehicle_fb;

    size_t count_;

    void timer_callback(){
        // auto t_start = std::chrono::steady_clock::now();

        steering->steering_feedback();
        auto steering_fb = std_msgs::msg::Float32MultiArray();

        // auto t_steering = std::chrono::steady_clock::now();
        // double steering_ms = std::chrono::duration<double, std::milli>(t_steering - t_start).count();

        steering_fb.data.push_back(steering->fb.pos);
        steering_fb.data.push_back(steering->fb.spd);
        steering_fb.data.push_back(steering->fb.cur);
        steering_fb.data.push_back(steering->fb.temp);
        steering_fb.data.push_back(steering->fb.err);

        pub_str_fb->publish(steering_fb);

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Steering feedback published");

        vehicle->read_feedback();
        auto vehicle_fb = std_msgs::msg::Float32MultiArray();

        // auto t_vehicle = std::chrono::steady_clock::now();
        // double vehicle_ms = std::chrono::duration<double, std::milli>(t_vehicle - t_steering).count();

        vehicle_fb.data.push_back(vehicle->fb.throttle);
        vehicle_fb.data.push_back(vehicle->fb.brake);
        // vehicle_fb.data.push_back(vehicle->fb.speed);
        // vehicle_fb.data.push_back(vehicle->fb.acceleration);
        vehicle_fb.data.push_back(vehicle->fb.motor_rpm);
        vehicle_fb.data.push_back(vehicle->fb.current);
        // vehicle_fb.data.push_back(vehicle->fb.sys_flags);
        // vehicle_fb.data.push_back(vehicle->fb.main_state);

        pub_vehicle_fb->publish(vehicle_fb);

        // RCLCPP_INFO(this->get_logger() , "Vehicle feedback published");
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        // // "Vehicle feedback: Thr:%d, Brk:%d, Spd:%d, Acc:%d, RPM:%d, Cur:%d, Flags:%d",
        // "Vehicle feedback: Thr:%d, Brk:%d, RPM:%f, Cur:%d",
        // vehicle->fb.throttle,
        // vehicle->fb.brake,
        // // vehicle->fb.speed,
        // vehicle->fb.motor_rpm,
        // vehicle->fb.current);

        // Log the times
        // RCLCPP_INFO(this->get_logger(), " Steering: %.2fms | Vehicle: %.2fms", 
        //         steering_ms, vehicle_ms);

    }

    void steering_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{

        // if takeover is initiated -- then we command 0 torque - and stop motor
        if (msg->linear.y == -1) steering->steering_stop();
        
        else steering->steering_command(msg->linear.x);

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "steering sent '%f'" , msg->linear.x);
    }

    void throttle_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{

        // if (msg->linear.x > 0)
        vehicle->send_drive_command((long)msg->linear.x , (long)msg->linear.y );
        // else vehicle->send_drive_command(0 , -1*(long)msg->linear.x);

        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "throttle sent '%f'" ,  msg->linear.x);
    }

};


int main(int argc , char * argv[]){

    std::string share_dir = ament_index_cpp::get_package_share_directory("vehicle_interface");
    std::string config_path = share_dir + "/config/can_config.yaml";

    // FLAGS_log_dir = share_dir+ "../../logs";
    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    can_handler driver(config_path);
    steering_can str( &driver , config_path);
    vehicle_can veh( &driver , config_path);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<vehicle_interface>(&str , &veh);

    // CHANGE: Use MultiThreadedExecutor instead of default spin
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;

}
