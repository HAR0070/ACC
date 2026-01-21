#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// Your Drivers
#include "radar_can.hpp"
#include "can.hpp"

using namespace std::chrono_literals;

class RadarPublisher : public rclcpp::Node {
public:
    RadarPublisher(can_handler * driver , radar_can* radar_can ) : can_driver_(driver),
                                                                    radar_driver_(radar_can),
                                                                    Node("radar_driver_node") {
        // publisher for PointCloud2
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/radar/points", 10);

        RCLCPP_INFO(this->get_logger(), "SR75 Radar Hardware Initialized.");

        // Timer loop at 20Hz (50ms) to poll CAN and publish
        timer_ = this->create_wall_timer(
            50ms, std::bind(&RadarPublisher::timer_callback, this));
    }

private:

    std::shared_ptr<can_handler> can_driver_;
    std::shared_ptr<radar_can> radar_driver_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback() {
        if (!radar_driver_) return;

        // This will populate radar_driver_->current_points with X, Y, Z, V, RCS
        radar_driver_->update_radar();

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "radar_link" ;   // fixed frame id for rviz


        cloud_msg.height = 1;
        cloud_msg.width = radar_driver_->current_points.size();  // number of points at each instance
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;


        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);  // to modify as a container
        modifier.setPointCloud2Fields ( 5,
        "x" , 1 , sensor_msgs::msg::PointField::FLOAT32,   // for one msg , there is 1 point of float32
        "y" , 1 , sensor_msgs::msg::PointField::FLOAT32,
        "z" , 1 , sensor_msgs::msg::PointField::FLOAT32,
        "intensity" , 1 , sensor_msgs::msg::PointField::FLOAT32,
        "velocity" , 1 , sensor_msgs::msg::PointField::FLOAT32);

        modifier.resize(radar_driver_->current_points.size());  // to ensure size


        // Now filling all the messages in  radar_driver_->current_points
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<float> iter_velocity(cloud_msg, "velocity");

        for (const auto &elem : radar_driver_->current_points){
            *iter_x = elem.x;   ++iter_x;
            *iter_y = elem.y;   ++ iter_y;
            *iter_z = elem.z;   ++iter_z;
            *iter_intensity = elem.rcs;   ++iter_intensity;
            *iter_velocity = elem.velocity;     ++iter_velocity;

        }

        publisher_->publish(cloud_msg);
    }

};

int main(int argc, char * argv[]) {
    // Initialize Google Logging (required by can.hpp)
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;

    std::string share_dir = ament_index_cpp::get_package_share_directory("radar_ros");
    std::string config_path = share_dir + "/config/can_config.yaml";

    can_handler driver(config_path);
    radar_can radar( &driver , config_path);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarPublisher>(&driver , &radar));
    rclcpp::shutdown();
    return 0;
}
