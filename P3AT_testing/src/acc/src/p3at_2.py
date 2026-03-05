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
import matplotlib.pyplot as plt
plt.ion()


flag = 0
front_distance = 0
prev_fd = 0
prev_vh = 0
vh = 0
v_rel = 0
dt = 0.1
ah = 0


class var():
    def __init__(self):
        self.x = 0 #  x and y positions from subscriber
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.a_cmd = np.array([]) # commanded acceleration and velocity
        self.v_cmd = np.array([])

        self.x0 = np.array([0,0,0]) # mpc states
        self.host = np.array([0,0,0]) # host states
        self.ref = np.array([0,0,0]) # host states
        self.y1 = np.array([0,0,0])
        self.t = 0
        self.u = 0
        self.dt = 0.1  # seconds
        var.tag =True
        var.iter = 0
        var.i = 0

def rnd(number, precision=3):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

def lagd(a, N):
    v = np.zeros(N)
    Lo = np.zeros(N)
    v[0] = a
    Lo[0] = 1

    for k in range(1, N):
        v[k] = ((-a) ** (k - 1)) * (1 - a * a)
        Lo[k] = (-a) ** (k)

    Lo = np.sqrt((1 - a * a)) * Lo
    # Lo = Lo.reshape(-1,1)
    A = np.zeros((N, N))
    A[:, 0] = v

    for i in range(1, N):
        A[:, i] = np.concatenate([np.zeros(i), v[0:N - i]])
    # print(Lo)
    return A, Lo

def dmpc(Ae, Be, a, N, Np, Q, R):

    n, _ = Be.shape
    n_in = 1
    Npa = N[0]  # dimention of eta = sum of all the values of Ns
    E = np.zeros((Npa,Npa))
    H = np.zeros((Npa,n))
    R_para = np.zeros((Npa,Npa))
    n0 = 0
    ne = N[0]
    for i in range(0,n_in):
        R_para[n0:ne,n0:ne] = R[i,i]*np.eye(N[i],N[i])

    # Now initial condition for the convolution sum
    Sin = np.zeros((n,Npa))

    [A1,Lo] = lagd(a[0],N[0])
    Lo = Lo.reshape(-1, 1)
    # print(Be.shape, Lo.shape)
    Sin[:,0:N[0]] = Be@np.transpose(Lo)

    Sc = Sin
    E = np.transpose(Sc)@Q@Sc
    H = np.transpose(Sc)@Q@Ae
## The iteration i is with respect to the prediction horizon
    for i in range(0,Np):
        Eae = np.linalg.matrix_power(Ae, i+1)
        [A1,Lo] = lagd(a[0],N[0])
        Lo = Lo.reshape(-1,1)
        Sc[:,0:N[0]] = np.linalg.matrix_power(Ae, i-1)@Sc[:,0:N[0]] + Be@np.transpose(np.linalg.matrix_power(A1, i-1)@Lo)
        E = E + np.transpose(Sc)@Q@Sc
        H = H + np.transpose(Sc)@Q@Eae
    E = E + R_para

    return E,H

def Mdu(a, N, Nc, n_in=1):
    N_pa = np.sum(N)
    M = np.zeros((n_in, N_pa))
    M_du1 = np.zeros((n_in, N_pa))
    k0 = 0
    Al, L0 = lagd(a[k0], N[k0])
    M_du1[k0, :N[k0]] = np.transpose(L0)
    cc = N[k0]

    for k0 in range(1, n_in):
        Al, L0 = lagd(a[k0], N[k0])
        M_du1[k0, cc:cc+N[k0]] = np.transpose(L0)
        cc += N[k0]

    Lzerot = np.copy(M_du1)
    M = np.copy(M_du1)

    for kk in range(2, Nc+1):
        k0 = 0
        Al, L0 = lagd(a[k0], N[k0])
        L = np.linalg.matrix_power(Al, kk-1) @ L0
        M_du1[k0, :N[k0]] = np.transpose(L)
        cc = N[k0]

        M = np.vstack((M, M_du1))

    return M, Lzerot

def QPHild(E, F, M, gamma):
    # inputs are H f A_cons b  in order
    # Determine which constraints are active and which are inactive
    n1 = M.shape[0]
    m1 = M.shape[1]
    eta = -np.linalg.solve(E, F)
    kk = 0

    for i in range(n1):
        if np.dot(M[i], eta) > gamma[i]:
            kk += 1

    if kk == 0:
        return eta

    P = np.dot(M,np.dot(np.linalg.inv(E),M.T))
    d = np.dot(M,np.dot(np.linalg.inv(E),F)) + gamma

    n, m = d.shape
    x_ini = np.zeros((n, m))
    lam = x_ini
    al = 10

    # 40 is the number of iterations to
    for km in range(40):
        lambda_p = np.copy(lam)

        for i in range(n):
            w = np.dot(P[i,:],lam) - np.dot(P[i,i],lam[i])
            w = w + d[i]
            la = -w/P[i,i]
            lam[i,0] = max(0,la)
            #print(lam[i,0])
        al = np.dot((lam-lambda_p).T,lam-lambda_p)

        if al < 10e-8:
            break

    eta = eta  - np.dot(np.linalg.solve(E, M.T), lam)
    return eta

def Mu(a, N, Nc, n_in=1):
    N_pa = np.sum(N)
    M = np.zeros((n_in, N_pa))
    M_du1 = np.zeros((n_in, N_pa))
    k0 = 0
    Al, L0 = lagd(a[k0], N[k0])
    M_du1[k0, :N[k0]] = np.transpose(L0)
    cc = N[k0]

    for k0 in range(1, n_in):
        Al, L0 = lagd(a[k0], N[k0])
        M_du1[k0, cc:cc+N[k0]] = np.transpose(L0)
        cc += N[k0]

    M1 = np.copy(M_du1)
    M = np.copy(M_du1)

    for kk in range(1, Nc):
        k0 = 0
        Al, L0 = lagd(a[k0], N[k0])
        L = np.linalg.matrix_power(Al, kk-1) @ L0
        M_du1[k0, :N[k0]] = np.transpose(L)
        cc = N[k0]

        M = M + M_du1
        M1 = np.vstack((M1, M))

    return M1
## Constrints are clamping type
def simCon3(xm, y, Ae, Be, Ce, N_sim, Omega, Psi, Lzerot, M0, M1):
    m1, n1 = Ce.shape
    u = np.zeros((1, 1))
    yr = np.array([[0], [0], [0]])
    # n1, _ = Bp.shape
    n_in = 1
    Lzerot = Lzerot.reshape(-1,1)

    u_max = 2
    u_min = -2
    delu_min = -2
    delu_max = 2
    Nc = 15   # number of steps on which limit has to be imposed
    M = np.vstack((M0,-M0,M1, -M1))


    u1 = np.zeros((N_sim, 1))
    y1 = np.zeros((m1, N_sim))
    deltau1 = np.zeros((N_sim,1))
    xf = np.vstack((xm, (y - yr)))

    for kk in range(N_sim):
        u_prev = u
        gamma = np.vstack(((u_max - u_prev) * np.ones((Nc, 1)),
                    -(u_min - u_prev) * np.ones((Nc, 1))
                    , delu_max * np.ones((Nc,1))
                    , -delu_min * np.ones((Nc,1))))
        # print(f"this is Omega {Omega.shape}, Psi {Psi.shape}, M {M.shape}, gamma {gamma.shape}")
        eta = QPHild(Omega, Psi@xf, M, gamma)
        # Kmpc = np.transpose(Lzerot) @ np.linalg.solve(Omega, Psi)
        # Ke = Kmpc[0,3:6].reshape(1,-1)
        deltau = np.transpose(Lzerot) @ eta   #
        if deltau[0] > delu_max:
            deltau[0] = delu_max
        if deltau[0] < delu_min:
            deltau[0] = delu_min
        u += deltau
        if u[0] > u_max:
            u[0] = u_max
        if u[0] < u_min:
            u[0] = u_min

        deltau1[kk] = deltau
        u1[kk] = u
        xm_old = xm.copy()
        xm = Ae@xm + Be@u
        y = Ce@xm

        plt.figure("computed accel")
        plt.scatter(kk,xm[2])
        # xf = (Ae - Be@Kmpc) @ xf + Be @ Ke @ yr #xf = Ae @ xf + Be @ deltau
        # y = Ce @ xf

        y1[:,kk] = y.reshape(3,)
        xf = np.vstack((xm - xm_old, y - yr)) #Xf = np.vstack((xm - xm_old, y - sp[:, kk + 1]))

    k = np.arange(N_sim)
    plt.figure("computed u")
    plt.plot(k,u1)


    return u1, y1, deltau1, k

def speed_filter():
    speed_filter.previousSpeed.append(var.vx)
    n = 5
    if len(speed_filter.previousSpeed) > n:  # keep only 5 values
        speed_filter.previousSpeed[:n]
    return sum(speed_filter.previousSpeed) / n
speed_filter.previousSpeed = []

def get_filtered_acel():
    """Filter the speed command to avoid abrupt speed changes."""
    get_filtered_acel.previousAcel = np.append(get_filtered_acel.previousAcel, var.vx)
    get_filtered_acel.prevTime =  np.append(get_filtered_acel.prevTime , var.t)
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

get_filtered_acel.previousAcel = np.array([0,0])
get_filtered_acel.prevTime = np.array([0,0])

var = var()


T_eng =  0.2 #0.26  #
K_eng = 0.6
A_f = -1/T_eng
B_f = K_eng/T_eng
C = np.eye(3)
T_hw = 0.15
d0 = 1
Ts = var.dt

# Discretize the system
A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B = np.array([[0], [0], [B_f]])
sys2 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A, B, C, D , dt = sys2

m1,n1 = C.shape  # m1 = 3, n1 = 3
n1,n_in = B.shape  # n_in = 1

a1 = 0.7  #0.6 and weight 10*Ce@Ce and R - 1, a = 5 - 7  ( 7 is slow, 5 is fast response)
N1 = 7
Np = 40

a = [a1]
N = [N1]

# Augment the state equations
Ae = np.eye(n1+m1,n1+m1)
Ae[0:n1,0:n1] = A
Ae[n1:n1+m1,0:n1] = C@A
Be = np.zeros((n1+m1,n_in))
Be[0:n1,:] = B
Be[n1:n1+m1,:] = C@B
Ce = np.zeros((m1,n1+m1))
Ce[:,n1:n1+m1] = np.eye(m1,m1)

# Q1 = np.transpose(C)@C

Q = 5*np.transpose(Ce)@Ce
R = 2*np.eye(1,1)

# Initialize variables
N_sim = Np
Nc = 15

var.x0 = np.array([[50], [10], [2]])
yr = np.array([[0], [0], [0]])
M1, Lzerot = Mdu(a, N,Nc,n_in)
M0 = Mu(a, N,Nc,n_in)
_, Lz = lagd(a[0], N[0])
Omega, Psi = dmpc(Ae, Be, a, N, Np, Q, R)

previous_message = ''

y1 = np.array([[50], [10], [2]])
path = [-100]
cmd = [[0]]
failure_iter  = 0

def pose_callback(data):
    print("this is odo call back")

rospy.init_node('P3_AT', anonymous=True)
pub = rospy.Publisher('/RosAria/cmd_vel',Twist, queue_size=10)
# scanner = rospy.Subscriber('/scan', LaserScan, lidar_callback)
odom = rospy.Subscriber('/RosAria/pose', Odometry, pose_callback)

def solve():
    d_c = var.x0[0] - d0 - T_hw*var.vx
    x0 = np.asarray([d_c, var.x0[1], var.x0[2]])
    x0 = x0.reshape(3,1)
    path , y , _ , _ = simCon3(x0,y1,A,B,C,N_sim,Omega,Psi,Lz,M0, M1)
    var.y1 = y[:,0].reshape(3,1)
    #print(f"this is y1 {y1}")

    cmd = []
    if path[0]!= -100 and len(path)>1:
        
        i=0
        for point in path:
            cmd.append(point[-1])
            i+=1
            if i>5:
                break
        var.tag = True
    else:
            cmd = -100
            failure_iter += 1
            print(f"this is failure iter {failure_iter}")

    var.a_cmd = np.array(cmd)
    var.tag = True

def pose_callback(data):
    var.vx = data.twist.twist.linear.x
    print("this is odo call back")

# def lidar_callback(data):
#         #data = LaserScan()
#         sum = 0
#         sum1 =0
#         k= 0
#         #print(data.ranges)
#         #for i in range(1):
#         #print(data.ranges)
#         sum = list(data.ranges)
#         #print(len(sum))
#         min_angle=data.angle_min
#         max_angle=data.angle_max
#         inc_angle=data.angle_increment
#         angles=np.arange(min_angle,max_angle,inc_angle)
#         angles=np.append(angles,max_angle)
#         polar_coordinates=list(zip(sum,angles))
#         #print(polar_coordinates[570])
#
#         for i in range(30):
#           inf = float('inf')
#           if sum[515+i] != inf:
#            sum1 += sum[515 + i]
#            k=k+1
#         average = sum1/k
#         print(average)
#         #print("Distance from obstacle is")
#         #print(average)
#         global front_distance
#         global prev_fd
#         global v_rel
#         prev_fd = front_distance
#         front_distance = average
#         prev_fdprev_fd = (average - front_distance)/dt
#         #solve(y1,A,B,C,N_sim,Omega,Psi,Lz,M0,M1)
#         flag = 1

def publish_cmd():
    rate = rospy.Rate(int(1/var.dt))  # this is in hertz
    #print("publishing")
    twist = Twist()
    if var.tag == True:
        var.iter = 0
    
    if var.iter <=5:
        ax = var.a_cmd[var.iter]
        var.v_cmd = var.vx + ax*var.dt
        twist.linear.x = var.v_cmd + 0.2
        i = var.i
        plt.figure("commanded velocity")
        plt.scatter(i,var.v_cmd)
        rospy.loginfo(var.v_cmd)
        print(f"published data {var.v_cmd}")
        pub.publish(twist)
        var.iter +=1

def main():
    y1 = var.x0.reshape(3,1)
    while not rospy.is_shutdown():
        if var.x0[0] < 0.2:
            break
        var.ref = np.array([5,0,0])
        try:
            data = rospy.wait_for_message('/RosAria/pose', Odometry, timeout=20)
            var.x = data.pose.pose.position.x
            var.y = data.pose.pose.position.y
            var.vx = data.twist.twist.linear.x
            var.vy = data.twist.twist.linear.y

            print(data)
        except rospy.ROSInterruptException:
            pass
        var.t = time.time()
        speed_filter()
        get_filtered_acel()
        var.host = np.array([var.x , var.vx , var.ax])
        var.x0 = np.array([(var.ref[0] - var.x) , (var.ref[1]-var.vx), var.ax])
        print(f"now going to solve {var.x0}")
        if var.tag == True:
            solve()
        publish_cmd()
        i = var.i
        plt.figure("del D")
        plt.scatter(i,var.x0[0])

        plt.figure("velocity")
        plt.scatter(i, var.vx)

        # plt.figure("accel")
        # plt.figure(i,var.x0[2])
        var.i +=1

        plt.show()
        plt.pause(0.001)
    rospy.spin()

if __name__ == '__main__':
    main()
