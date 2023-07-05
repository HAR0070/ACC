#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import sys
from pynput import keyboard
from pynput.keyboard import Key
import threading

twist = Twist()
prev = None

def on_press(key):
    global prev
    if key.char == 'w':
        # print("This is w now")
        prev = 'w'
    elif key.char == 's':
        # print("This is s now")
        prev = 's'
    elif key.char == '.':
        prev = '.'
    else:
        prev = 0

def on_release(key):
    prev = 0
    pass

def run():
    '''Start monitoring the keys.'''
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

run_thread = threading.Thread(target=run)
run_thread.start()

def cmd_gen(v, u_cmd, u):
    toe = 0.5
    k = 0.7
    acel = u_cmd*0.90 + (k*u)*dt/toe
    v_cmd = v + acel*dt

    if v_cmd > 2 :
        v_cmd = 2
    elif v_cmd <0:
        v_cmd = 0

    return v_cmd , u_cmd


def values(u_cmd):
    # print("w for forward, s for reverse, and . to exit" + '\n')
    s = prev
    if s == 'w':
        twist.linear.x , u_cmd = cmd_gen(twist.linear.x,u_cmd, 1.0)
        print(f"twist is updated {twist.linear.x}")
        twist.angular.z = 0.0
        twist.linear.y = 0.0
    elif s == 's':
        twist.linear.x , u_cmd = cmd_gen(twist.linear.x,u_cmd, -1.0)
        print(f"twist is update top {twist.linear.x}")
        twist.angular.z = 0.0
        twist.linear.y = 0.0
    elif s == '.':
        twist.angular.z = twist.linear.x = twist.linear.y = 0.0
        sys.exit()
    return twist , u_cmd

def keyboard():
    u_cmd = 0
    pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10) # que is FIFO
    rospy.init_node('teleop_py',anonymous=True)
    rate = rospy.Rate(1/dt)
    while not rospy.is_shutdown():
        twist, u_cmd = values(u_cmd)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    dt = 0.1
    try:
        keyboard()
    except rospy.ROSInterruptException:
        pass

#

# vel=Twist()
# def main():
#     rospy.init_node('astar_vel_pub')
#     pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#     rate = rospy.Rate(10) # 10hz
#     vel.linear.x= 0.1
#     a = 0
#     while not rospy.is_shutdown():
#         a +=1
#         if a < 100:
#             vel.linear.x= vel.linear.x + 0.1
#         else:
#             vel.linear.x= 0
#         rospy.loginfo(vel)
#         pub.publish(vel)
#         rate.sleep()
#
#
# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass
