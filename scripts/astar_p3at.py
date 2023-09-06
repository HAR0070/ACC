#!/usr/bin/env python3
# license removed for brevity


import numpy as np
import pandas as pd
import math
import time
import csv

import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class var():
    def __init__(self):
        self.x = 0 #  x and y positions from subscriber
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.a_cmd = 0 # commanded acceleration and velocity
        self.v_cmd = 0

        self.x0 = np.array([0,0,0]) # mpc states
        self.host = np.array([0,0,0]) # host states
        self.ref = np.array([0,0,0]) # host states
        self.t = 0
        self.u = 0
        self.dt = 0.1  # seconds

"""
##  Publisher
"""

def values(v):
    # print("w for forward, s for reverse, and . to exit" + '\n')
    print(" pub")
    twist.linear.x = v
    return twist

def publish_cmd(v):
    pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10) # que is FIFO
    rate = rospy.Rate(1/var.dt)  # this is in hertz
    # print("publishing")
    # while not rospy.is_shutdown():
    twist = values(v)
    pub.publish(twist)
    rate.sleep()

"""
Astar algo
"""
class States:
    def __init__(self):
        self.data = {}

    def add(self, id, value):
        if len(value) !=6:
            raise ValueError("Values must be a list of 6 elements")
        self.data[id] = value

    def get(self, id):
        return self.data.get(id)

    def x(self, id):
        return self.data.get(id)[0]

    def v(self, id):
        return self.data.get(id)[1]

    def a(self, id):
        return self.data.get(id)[2]

    def parent_id(self,id):
        return self.data.get(id)[3]

    def cost(self,id):
        return self.data.get(id)[4]

    def input(self,id):
        return self.data.get(id)[5]

    def len(self):
        return len(self.data)

    def key(self):
        return self.data.keys()

    def rmv_id(self,id):
        del self.data[id]

    def remove(self):
        self.data = dict(sorted(self.data.items(), key=lambda x: x[1][4]))
        key = next(iter(self.data))
        value = self.data.pop(key)
        return key, value

def propogation(u,state_p, plot = False):
    # Initialization
    step_t = 1
    dt = 0.1
    N = int(step_t/dt)
    x = np.zeros((N+1,1))
    t = np.zeros((N+1,1))
    x_dot = state_p[1]
    x_doubledot = state_p[2]
    x[0] = state_p[0]
    t[0] = 0
    k = 0.743
    T = 0.532
    for i in range(N):
        # Compute the jerk (third derivative of x) using the equation
        # Update the position, velocity, and acceleration using Euler's method
        x[i+1] = x[i] + x_dot*dt + x_doubledot*dt
        x_dot += x_doubledot * dt
        x_doubledot = x_doubledot*0.90 + (k * u)*dt/T
        # Store or use the generated state x
        t[i+1] = t[i] + dt
    if plot:
        plt.scatter(x,t)
        plt.show()
        plt.pause(0.001)
    return x , x[-1]-x[0], x_doubledot, x_dot

def is_goal(neighbor, goal, do, T_hw, res):
    # goal will be x coordinate with a and v
    diff1 = (goal[0] - neighbor[0])**2
    diff2 = 2*(goal[1] - neighbor[1])**2
    diff3 = (goal[2] - neighbor[2])**2
    res = 0.2
    ########
    #
    # correct the resolution later
    #
    #######
    if (diff3 < res) and (diff2 < res) and (diff1 < res):
        return True

def astar_v(start, goal, do, T_hw, resolution):
    # print("now running a star")
    parent = []
    id = 0
    Ofringe = States()
    node = [*start, parent, 0 , 0]
    Ofringe.add(id, node)
    Cfringe = States()
    path = []
    y = 0
    if is_goal(start, goal, do, T_hw, resolution):
        return path

    b = 0
    jerk = [i * 0.3 for i in range(-5, 5)]
    while Ofringe.len() >0:
        b +=1
        key, node = Ofringe.remove()
        parent_id = key
        Cfringe.add(key, node)

        for u in jerk:
            state_p = node # copy of the parent node

            if abs(state_p[2] - u) > 1.5:
                continue

            trajectory, ds , var.a_cmd , v = propogation(u, state_p) # add false
            neighbor = [trajectory[-1][0] , v, var.a_cmd]

            if var.a_cmd > 2 or var.a_cmd < -3.5:
                continue

            if ds < 0 or v < 0:
                continue

            if neighbor[0] > goal[0]:
                continue

            if is_goal(neighbor, goal, do, T_hw, resolution):
                for id in reversed(state_p[3]):
                     print( Cfringe.v(id))
                var.v_cmd = Cfringe.v(state_p[3][1])
                return

            h1 = 0
            h2 = 0
            cost1 = (goal[0]- neighbor[0])**2 # + do
            cost2 = (var.a_cmd)**2  #+ 1/(ds + 0.1) + 2*jerk**2
            cost3 = (goal[1] - neighbor[1])**2
            cost = np.round((cost2*2+cost3*1.2+ cost1*1.1), 3)

            tag = True
            res = resolution**2
            for key in Ofringe.key():
                diff1 = (neighbor[0] - Ofringe.x(key))**2
                diff2 = (neighbor[1] - Ofringe.v(key))**2
                diff3 = (neighbor[2] - Ofringe.a(key))**2
                if (diff3 < res) and (diff2 < res) and (diff1 < res):
                    # print("node comes inside resolution")
                    tag = False
                    if state_p[4] < Ofringe.cost(key):
                        id+=1
                        nodea = [*neighbor, [*node[3], parent_id], cost+ node[4], u]
                        Ofringe.add(id,nodea)
                        Ofringe.rmv_id(key)
                        break
                    else:
                        break
                else:
                    continue
            if tag:
                id+=1
                nodeb = [*neighbor, [*node[3], parent_id], cost+ node[4], u]
                Ofringe.add(id,nodeb)

    return 'brake'

"""
logging
"""

def logging(csv_file):
    with open(csv_file, mode='a') as file:
        writer = csv.writer(file)
        if file.tell() == 0:    # tells the current position of the file
            writer.writerow(['t','hx', 'hv', 'ha','rx', 'rv', 'ra','u','a_cmd','v_cmd'])

        var.t = time.time()
        writer.writerow([var.t,*var.host , *var.ref, var.u, var.a_cmd, var.v_cmd])

def global_goal(host, ref):
    diff = (host[0] - ref[0])**2 + (host[1] - ref[1])**2
    if diff < 0.5:
        return True

"""
main()
"""
twist = Twist()
var = var()
rospy.init_node('Astar', anonymous=True)
csv_file = "/home/har/catkin_ws/src/raspi/logs/astar2.csv"

print("Starting the astar controller ")

d0 = 0
T_hw = 1.5
resolution = np.sqrt(0.01)

var.host = np.array([0, 0.0, 0])

tag = False
# print("for loop")
while not tag:
    # print("this is",i)
    var.ref = np.array([4,0,0])
    # print("1")
    try:
        # listen()
        data = rospy.wait_for_message('/RosAria/pose', Odometry, timeout=10)
        var.x = data.pose.pose.position.x
        var.y = data.pose.pose.position.y
        var.vx = data.twist.twist.linear.x
        var.vy = data.twist.twist.linear.y

    except rospy.ROSInterruptException:
        pass
    var.host = np.array([var.x , var.vx , var.ax])  # position of host, velocity and commanded acel
    astar_v(var.host, var.ref, d0, T_hw, resolution)
    # print("3")
    try:
        publish_cmd(var.v_cmd)
        # print(var.v_cmd)
    except rospy.ROSInterruptException:
        pass

    tag = global_goal(var.host, var.ref)
    print(var.x,var.vx,var.a_cmd, var.v_cmd)
    logging(csv_file)

if tag:
    var.v_cmd = 0
    publish_cmd(var.v_cmd)
