#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import pandas as pd
from nav_msgs.msg import Odometry
from scipy.signal import cont2discrete
import time
import sys
import math

def publish_cmd(v):
        global pub
        dt =0.1
        rate = rospy.Rate(1/dt)  # this is in hertz
        #print("publishing")
        twist = Twist()
        twist.linear.x = v
        rospy.loginfo(v)
        pub.publish(twist)
        #rate.sleep()

rospy.init_node('P3_AT', anonymous=True)
pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10)
while not rospy.is_shutdown():

        publish_cmd(5)
