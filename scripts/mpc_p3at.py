#!/usr/bin/env python3
# license removed for brevity

import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import time
import csv
import math
from cvxpy import *
import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')
from scipy.signal import cont2discrete , tf2ss
import time
# import sys


plt.ion()
#####
#  Class to store all variabels
#####

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

######
# Subscribing to the p3at states
######

def callback(data):
    print(" callback")
    ## reading the pose message = positions

def listen():
    rospy.Subscriber('/RosAria/pose', Odometry, callback)
    # rospy.Subscriber('/RosAria/velocity ', Twist, callback_vel)
    print("listening")
    rospy.spin()

"""
check the nodes name which releases the velocity info

"""

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
    print("publishing")
    # while not rospy.is_shutdown():
    twist = values(v)
    pub.publish(twist)
    rate.sleep()

"""
# MPC algo
"""

def cvxpy_reverse2(A, B, N, Q, R, P, x0, xr, umax=None, umin=None, xmin=None, xmax=None):
    is_sol =  False
    (nx, nu) = B.shape

    # mpc calculation  x0 = distance, x1 = velocity, x2 = accel
    x = cvxpy.Variable((nx, N + 1))
    u = cvxpy.Variable((nu, N))

    costlist = 0.0
    constraints = []

    for t in range(N):

        costlist +=  cvxpy.quad_form(x[:, t], Q)
        costlist += cvxpy.quad_form(u[:, t], R)

        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]

        if xmin is not None:
            constraints += [x[0, t] >= xmin[0, 0]]  # state is greater than x min
            constraints += [x[2, t] >= xmin[1, 0]]
        if xmax is not None:
            constraints += [x[2, t] <= xmax[0]] # state is less than x max
        if umax is not None:
            constraints += [u[:, t] <= umax]  # input constraints
        if umin is not None:
            constraints += [u[:, t] >= umin]  # input constraints

    costlist +=  cvxpy.quad_form(x[:, N], P)

    constraints += [x[:, 0] == x0]  # inital state constraints

    ur1_max = 2
    ur1_min = -2

    prob = cvxpy.Problem(cvxpy.Minimize(costlist), constraints)

    prob.solve(solver=ECOS ) #solver= OSQP, ECOS, , warm_start=True

    if prob.status == 'optimal':
    # Problem was solved successfully
        is_sol = True
        print("Optimal solution found!")
    elif prob.status == 'infeasible':
        # Problem is infeasible (no feasible solution exists)
        print("Problem is infeasible.")
    elif prob.status == 'unbounded':
        # Problem is unbounded (no finite optimal solution exists)
        print("Problem is unbounded.")
    else:
        # Solver failed or terminated prematurely
        print("Solver failed or terminated prematurely. Status:", prob.status)

    return is_sol , x.value, u.value[0,0], costlist.value

def reverse(host , ref):
    N = 10  # horizon
    b = 0

    x_r = ref # is reference vehicle parameters
    d_ref = x_r[0] - (host[0] + T_hw*host[1] + d0)
    v_ref = x_r[1] - host[1]   # current values

    var.x0 = np.array([d_ref, v_ref, host[2]]) # referance state
    xr = np.array([0, 0 , 0]) # referance state
    try:
        succ, x, u, c = cvxpy_reverse2(A, B, N, Q, R, P, x0, xr, umax = umax, umin = umin, xmin = xmin, xmax = xmax)
    except:
        u = umin #  u_hist[-1] #
        print("not sucess")

    x0 = A.dot(var.x0) + B.dot(u)
    var.u = u
    try:
        if len(x0[0]) >1:
            x0 = x0[0]
    except:
        pass

    var.a_cmd = x0[2]
    var.v_cmd = host[1] + var.a_cmd*var.dt

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

def is_goal(host, ref):
    diff = (host[0] - ref[0])**2 + (host[1] - ref[1])**2
    if diff < 0.5:
        return True

def ploting(host):
    plt.scatter(host[0], host[1])

"""
filter
"""
def get_filtered_acel(acel, t):
    """Filter the speed command to avoid abrupt speed changes."""
    get_filtered_acel.previousAcel = np.append(get_filtered_acel.previousAcel, acel)
    get_filtered_acel.prevTime =  np.append(get_filtered_acel.prevTime , t)
    # print("this is prevTime" , get_filtered_acel.prevTime)
    # print("this is prevAcel" , get_filtered_acel.previousAcel)
    if len(get_filtered_acel.previousAcel) > 10:  # keep only 10 values
        get_filtered_acel.previousAcel = get_filtered_acel.previousAcel[1:]
        get_filtered_acel.prevTime = get_filtered_acel.prevTime[1:]

    sum = np.sum(np.diff(get_filtered_acel.previousAcel))
    t_diff = np.sum(np.diff(get_filtered_acel.prevTime))
    var.ax = sum / t_diff
    if math.isnan(ah):
        var.ax = 0

"""
main()
"""
twist = Twist()
var = var()
rospy.init_node('MPC', anonymous=True)
csv_file = "/home/har/catkin_ws/src/raspi/logs/mpc_test6.csv"
get_filtered_acel.previousAcel = np.array([0,0])
get_filtered_acel.prevTime = np.array([0,0])

print("Starting the MPC controller ")
T_eng =  0.460 #0.26  #
K_eng = 0.732
A_f = -1/T_eng
B_f = K_eng/T_eng
C = np.eye(3)
T_hw = 3
Ts = 0.05
T_total = 30
T = int(T_total/Ts)
d0 = 2

# print("discrete system")
# Discretize the system
A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B = np.array([[0], [0], [B_f]])
sys2 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A, B, C, D , dt = sys2

(nx, nu) = B.shape

Q = np.array([[1, 0, 0],[0,0.8,0],[0,0,0.5]])
R = np.eye(nu)*0.7
P = np.eye(nx)*0.1

## x represents states

umax = 3
umin = -3
amax = 2
amin = -2

# print("xmin, xmax")
xmin = np.array([[0] ,[amin]])  # state constraints
xmax = np.array([amax])  # state constraints

var.host = np.array([0, 0.0, 0])

tag = False
# print("for loop")
while not tag:
    # print("this is",i)
    var.ref = np.array([4.46,0,0])
    # print("1")
    try:
        # listen()
        data = rospy.wait_for_message('/RosAria/pose', Odometry, timeout=10)
        var.x = data.pose.pose.position.x
        var.y = data.pose.pose.position.y
        var.vx = data.twist.twist.linear.x
        var.vy = data.twist.twist.linear.y

        print(data)
        # print("2")
        # listen_vel()
    except rospy.ROSInterruptException:
        pass
    var.host = np.array([var.x , var.vx , var.ax])  # position of host, velocity and commanded acel
    reverse(var.host , var.ref)
    # print("3")
    try:
        publish_cmd(var.v_cmd)
        # print(var.v_cmd)
    except rospy.ROSInterruptException:
        pass

    tag = is_goal(var.host, var.ref)
    logging(csv_file)
    ploting(var.host)

if tag:
    var.v_cmd = 0
    publish_cmd(var.v_cmd)
