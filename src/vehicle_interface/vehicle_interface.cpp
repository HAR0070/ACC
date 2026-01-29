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


// Steering and throttle command come as twist
// both feedback go as float 32 array

class vehicle_interface : public rclcpp::Node
{

public:
    vehicle_interface(steering_can * str , vehicle_can* veh) : steering(str) ,
                                                                vehicle(veh),
                                                                Node("vehicle_interface"),
                                                                count_(0){

        pub_str_fb = this->create_publisher<std_msgs::msg::Float32MultiArray>("/steering_feedback" , 10);
        pub_vehicle_fb = this->create_publisher<std_msgs::msg::Float32MultiArray>("/vehicle_feedback" , 10);

        sub_str_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
                "/steering_pub" , 10 , std::bind(&vehicle_interface::steering_callback , this , std::placeholders::_1));

        sub_veh_cmd = this->create_subscription<geometry_msgs::msg::Twist>(
            "/vehicle_throttle" , 10 , std::bind(&vehicle_interface::throttle_callback, this , std::placeholders::_1));

        timer_ = this->create_wall_timer(
            50ms , std::bind(&vehicle_interface::timer_callback, this));  // full loop runs in 20Hz

    }

private:
    steering_can* steering;
    vehicle_can* vehicle;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_str_cmd;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_veh_cmd;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_str_fb;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_vehicle_fb;

    size_t count_;

    void timer_callback(){

        steering->steering_feedback();
        auto steering_fb = std_msgs::msg::Float32MultiArray();

        steering_fb.data.push_back(steering->fb.pos);
        steering_fb.data.push_back(steering->fb.spd);
        steering_fb.data.push_back(steering->fb.cur);
        steering_fb.data.push_back(steering->fb.temp);
        steering_fb.data.push_back(steering->fb.err);

        pub_str_fb->publish(steering_fb);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Steering feedback %d %d %d %d %d",
            steering_fb.data[0],
            steering_fb.data[1],
            steering_fb.data[2],
            steering_fb.data[3],
            steering_fb.data[4] );

        vehicle->read_feedback();
        auto vehicle_fb = std_msgs::msg::Float32MultiArray();

        vehicle_fb.data.push_back(vehicle->fb.throttle);
        vehicle_fb.data.push_back(vehicle->fb.brake);
        vehicle_fb.data.push_back(vehicle->fb.speed);
        vehicle_fb.data.push_back(vehicle->fb.acceleration);
        vehicle_fb.data.push_back(vehicle->fb.motor_rpm);
        vehicle_fb.data.push_back(vehicle->fb.current);
        vehicle_fb.data.push_back(vehicle->fb.sys_flags);
        vehicle_fb.data.push_back(vehicle->fb.main_state);

        pub_vehicle_fb->publish(vehicle_fb);

        // RCLCPP_INFO(this->get_logger() , "Vehicle feedback published");
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Vehicle feedback: Thr:%d, Brk:%d, Spd:%d, Acc:%d, RPM:%d, Cur:%d, Flags:%d",
        vehicle->fb.throttle,
        vehicle->fb.brake,
        vehicle->fb.speed,
        vehicle->fb.acceleration,
        vehicle->fb.motor_rpm,
        vehicle->fb.current,
        vehicle->fb.sys_flags);

    }

    void steering_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{
        steering->steering_command(msg->linear.x);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "steering sent '%f'" , msg->linear.x);
    }

    void throttle_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const{

        if (msg->linear.x > 0) vehicle->send_drive_command((long)msg->linear.x , 0);
        else vehicle->send_drive_command(0 , -1*(long)msg->linear.x);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "throttle sent '%f'" ,  msg->linear.x);
    }

};


int main(int argc , char * argv[]){

    std::string share_dir = ament_index_cpp::get_package_share_directory("vehicle_interface");
    std::string config_path = share_dir + "/config/can_config.yaml";

    // FLAGS_log_dir = share_dir+ "../../logs";
    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);

    can_handler driver(config_path);
    LOG(INFO) << "Initialized the can driver";

    steering_can str( &driver , config_path);
    LOG(INFO) << "Initialized the steering driver";

    vehicle_can veh( &driver , config_path);

    LOG(INFO) << " Initialized the vehicle";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vehicle_interface>(&str , &veh));
    rclcpp::shutdown();
    return 0;

}
