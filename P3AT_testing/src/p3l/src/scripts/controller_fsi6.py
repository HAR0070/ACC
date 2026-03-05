#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
    # Author: Andrew Dai
    # This ROS Node converts Joystick inputs from the joy node
    # into commands for turtlesim

    # Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
twist1 = Twist()
twist2=Twist()
rotation=1
Automode=False

def autonomous():
    global rotation
    global timerem
    print("time rem:",timerem-time.time())
    #print("Current time:",time.time())
    if((rotation==1)):
        rospy.loginfo('Rotating in clockwise')
        if(timerem-time.time()<=0):
            rotation=-1
            timerem=time.time()+5
        autovelw=0.25
    elif(rotation==-1):
        rospy.loginfo('Rotating Anticlockwise')
        if(timerem-time.time()<=0):
            rotation=1
            timerem=time.time()+5
        autovelw=-0.25
    else:
        autovelw=0

    twist2.angular.z=autovelw

    

def ch1_callback(msg):
    
    stickpose1=msg.data

    if(stickpose1>=1500):
        velw=-0.25
    elif(stickpose1>=1450 and stickpose1<1500):
        velw=0
    elif(stickpose1<1450 and stickpose1>=900):
        velw=0.25
    else:
        velw=0
        rospy.loginfo("No signal from channel 1")


    #velw=-0.005*(stickpose1-1500)
    twist1.angular.z=velw
    #pub.publish(twist)

def ch2_callback(msg):

    stickpose1=msg.data

    if(stickpose1>=1500):
        velx=0.35
    elif(stickpose1>=1450 and stickpose1<1500):
        velx=0
    elif(stickpose1<1450 and stickpose1>=900):
        velx=-0.35
    else:
        velx=0
        rospy.loginfo("No signal from channel 2")

    twist1.linear.x = velx
    #pub.publish(twist)

def ch5_callback(msg):
    
    global Automode

    stickpose1=msg.data

    if(stickpose1>=1500):
        Automode=True
    elif(stickpose1>=800 and stickpose1<1500):
        Automode=False
    elif(stickpose1>=0):
    	Automode=False
    	rospy.loginfo("No signal from channel5")
    else:
        Automode=False
        rospy.loginfo("Serial node is shutdown. Turn it on")



    # Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    global timerem
    rospy.init_node('fsi6_controller')
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist)
    rate=rospy.Rate(10)

    timerem=time.time()+5

    rospy.Subscriber("ch1", Float32, ch1_callback)
    rospy.Subscriber("ch2", Float32, ch2_callback)
    rospy.Subscriber("ch5", Float32, ch5_callback)

    while not rospy.is_shutdown():
        if(Automode):
            autonomous()
            pub.publish(twist2)
        else:
            pub.publish(twist1)
        rate.sleep()
    


    # starts the node
    rospy.spin()

if __name__ == '__main__':
    start()
