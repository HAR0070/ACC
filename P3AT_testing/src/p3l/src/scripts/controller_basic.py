#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

    # Author: Andrew Dai
    # This ROS Node converts Joystick inputs from the joy node
    # into commands for turtlesim

    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
    
    
    
def callback(data):
    
    print(data)
    mapping = {'up':1, 'down':-1, 'left':-1}
    
    twist.linear.x = 0.35*data.axes[1]
    twist.linear.y = 0.01*data.axes[4]
    twist.angular.z = 0.25*data.axes[0]
    
    print(twist)
    

    # Intializes everything
def start():
        # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    global twist 
    twist = Twist()
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist,queue_size=1)
        # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
        # starts the node
    rospy.init_node('JoyARia')
    print(twist)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    	#print(twist) 
    	pub.publish(twist)
    	rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    start()
