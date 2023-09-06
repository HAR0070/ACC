#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

vel=Twist()
def main():
    rospy.init_node('astar_vel_pub')
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel.linear.x= 0.1
    a = 0
    while not rospy.is_shutdown():
        a +=1
        if a < 100:
            vel.linear.x= vel.linear.x + 0.1
        else:
            vel.linear.x= 0
        rospy.loginfo(vel)
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
