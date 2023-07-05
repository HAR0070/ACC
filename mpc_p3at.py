#!/usr/bin/env python3
# license removed for brevity
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
# import sys
# from pynput import keyboard
# from pynput.keyboard import Key
# import threading
import csv



#####
#  Class to store all variabels
#####

class var():
    def __init__(self):
        self.x = 0 #  x and y positions from subscriber
        self.y = 0
        self.vx = 0
        self.vy = 0
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
    ## reading the pose message = positions
    var.x, var.y = data.linear.x, data.linear.y

def callback_vel(data):
    ## reading the pose message = positions
    var.vx, var.vy = data.linear.x, data.linear.y

def listen_pose():
    rospy.init_node('MPC', anonymous=True)
    rospy.Subscriber('/RosAria/pose', Twist, callback)
    rospy.Subscriber('/RosAria/velocity ', Twist, callback_vel)
    rospy.spin()

"""
check the nodes name which releases the velocity info

"""

############
##  Publisher
#############
def values(v):
    # print("w for forward, s for reverse, and . to exit" + '\n')
    twist.linear.x = v
    twist.angular.z = 0.0
    twist.linear.y = 0.0
    return twist

def publish_cmd(v):
    pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10) # que is FIFO
    rate = rospy.Rate(1/var.dt)  # this is i hertz
    while not rospy.is_shutdown():
        twist = values(v)
        pub.publish(twist)
        rate.sleep()

###############
# MPC algo
###############

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

    var.x0 = np.array([d_ref, v_ref, x0[2]]) # referance state
    xr = np.array([0, 0 , 0]) # referance state
    try:
        succ, x, u, c = cvxpy_reverse2(A, B, N, Q, R, P, x0, xr, umax = umax, umin = umin, xmin = xmin, xmax = xmax)
    except:
        u = umin #  u_hist[-1] #

    x0 = A.dot(x0) + B.dot(u)
    try:
        if len(x0[0]) >1:
            x0 = x0[0]
    except:
        pass

    # make the command
    var.a_cmd = x0[2]
    return(host[1] + var.a_cmd*var.dt)

"""

logging

"""

def logging():
    with open(csv_file, mode='a') as file:
        writer = csv.writer(file)
        if file.tell() == 0:    # tells the current position of the file
            writer.writerow(['x', 'y', 'vx','vy'])

        writer.writerow([])


twist = Twist()
var = var()

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

xmin = np.array([[0] ,[amin]])  # state constraints
xmax = np.array([amax])  # state constraints

var.host = np.array([0, 0.0, 0])
while True:
    var.ref = np.array([10,0,0])
    try:
        listen_pose()
        listen_vel()
    except rospy.ROSInterruptException:
        pass
    var.host = np.array([var.x , var.vx , var.a_cmd])  # position of host, velocity and commanded acel
    var.v_cmd = reverse(host , ref)
    try:
        publish_cmd(var.v_cmd)
    except rospy.ROSInterruptException:
        pass
