#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


def filter_laser_scan(scan_msg, start_angle, end_angle):
    filtered_ranges = scan_msg.ranges[int(start_angle / scan_msg.angle_increment):int(end_angle / scan_msg.angle_increment) + 1]

    filtered_scan_msg = LaserScan()
    filtered_scan_msg.header = scan_msg.header
    filtered_scan_msg.angle_min = start_angle
    filtered_scan_msg.angle_max = end_angle
    filtered_scan_msg.angle_increment = scan_msg.angle_increment
    filtered_scan_msg.time_increment = scan_msg.time_increment
    filtered_scan_msg.scan_time = scan_msg.scan_time
    filtered_scan_msg.range_min = scan_msg.range_min
    filtered_scan_msg.range_max = scan_msg.range_max
    filtered_scan_msg.ranges = filtered_ranges

    return filtered_scan_msg

def lidar_callback(msg):
    # print("I am subscribing")
    # Specify the desired angle range (in radians)
    start_angle = 00.0 * (3.141592653589793 / 180.0)  # Convert degrees to radians
    end_angle = 3.0 * (3.141592653589793 / 180.0)

    # Filter the LaserScan data
    filtered_scan_msg = filter_laser_scan(msg, start_angle, end_angle)
    # rate = rospy.Rate(20)
    # Publish the filtered LaserScan data
    pub.publish(filtered_scan_msg)

if __name__ == '__main__':
    rospy.init_node('Lidar')

    # Create a ROS publisher for the filtered LaserScan data
    pub = rospy.Publisher('/Lidar/filtered_laser_scan', LaserScan, queue_size=20)
    pub_rate = rospy.Rate(10)
    # Create a ROS subscriber for the original LaserScan data
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    pub_rate.sleep()
    rospy.spin()
