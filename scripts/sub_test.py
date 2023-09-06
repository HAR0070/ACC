#!/usr/bin/env python3
import rospy
import csv
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# twist = Twist()
# csv_file = "/home/har/catkin_ws/src/raspi/logs/test2.csv"

def callback(data):
    # print("this is pose values")
    # print(data)
    print(f" this is pose {data.pose.pose.position.x}, {data.pose.pose.position.y} ,this is vel {data.twist.twist.linear.x}, {data.twist.twist.linear.y}") #,
    # with open(csv_file, mode='a') as file:
    #     writer = csv.writer(file)
    #     if file.tell() == 0:    # tells the current position of the file
    #         writer.writerow(['linx', 'liny', 'linz'])
    #
    #     writer.writerow([data.linear.x, data.linear.y, data.linear.z])

def callback2(data):
    print("this is cmd_vel")
    print(data)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/RosAria/cmd_vel',Twist , callback) #Odometry
    # rospy.Subscriber('/RosAria/cmd_vel', Odometry, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
