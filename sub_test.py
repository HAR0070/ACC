#!/usr/bin/env python3
import rospy
import csv
from std_msgs.msg import String
from geometry_msgs.msg import Twist


twist = Twist()
csv_file = "/home/har/catkin_ws/src/raspi/logs/accel_test.csv"

def callback(data):
    with open(csv_file, mode='a') as file:
        writer = csv.writer(file)
        if file.tell() == 0:    # tells the current position of the file
            writer.writerow(['Linear X', 'Linear Y', 'Linear Z'])

        writer.writerow([data.linear.x, data.linear.y, data.linear.z])

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/RosAria/pose', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
