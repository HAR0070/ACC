#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def talker():
    angle = 0.3 * math.cos(driver.getTime())
    pub = rospy.Publisher('steeringAngle', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(angle)
        pub.publish(hello_str)
        rate.sleep()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.data))
    print(data.data)


if __name__ == '__main__':
    rospy.Subscriber("current_speed", Float64, callback)

    try:
        talker()

    except rospy.ROSInterruptException:
        pass
