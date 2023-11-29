""" Supervisor which runs the optimization """

"""
reciver listens to channel 5
emitter is on channel 4

Data is recived in the same order as it is sent
so the timings should be same
"""

import numpy as np
import warnings
warnings.filterwarnings('ignore')
from scipy.signal import cont2discrete
import control.matlab as cm
import time
import sys


from controller import Supervisor

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
    #Ae and Be are the matrices of the augmented system when integrator is used
    # They can also be other forms of state space model
    # a is the parameter of the laguerre network
    # N is the number of terms for each input
    # Np is the prediction horizon
    # Q and R are the cost matrices

    #cost function is J = eta^T E eta + 2 eta^T H x(k_i)
    #Δui(k) = Li(k)T ηi,

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

def exp_matrix(size, alpha):
    I = np.zeros((size,size))
    for i in range(0,size):
        I[i,i] = pow(alpha,i)

    return I

## Constrints are clamping type
def simCon3(xm, u, y, yr, A1, A2, B1 , B2, C, N_sim, Omega1, Omega2, Psi1, Psi2, Lzerot, M0, M1, I):
    m1, n1 = C.shape
    # n1, _ = Bp.shape
    n_in = 1
    Lzerot = Lzerot.reshape(-1,1)

    u_max = 2.5
    u_min = -3.5
    delu_min = -5
    delu_max = 3.5
    Nc = 15   # number of steps on which limit has to be imposed
    M = np.vstack((M0,-M0,M1, -M1))
    M = M@I

    u1 = np.zeros((n_in, N_sim))
    y1 = np.zeros((m1, N_sim))
    deltau1 = np.zeros((n_in, N_sim))
    # xf = np.vstack((xm, (y - yr)))

    for kk in range(N_sim):
        u_prev = u
        gamma = np.vstack(((u_max - u_prev) * np.ones((Nc, 1)),
                    -(u_min - u_prev) * np.ones((Nc, 1))
                    , delu_max * np.ones((Nc,1))
                    , -delu_min * np.ones((Nc,1))))
        # print(f"this is Omega {Omega.shape}, Psi {Psi.shape}, M {M.shape}, gamma {gamma.shape}")
        if u >=0:
            eta = QPHild(Omega1, Psi1@xm, M, gamma)
        elif u < 0:
            eta = QPHild(Omega2, Psi2@xm, M, gamma)
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

        deltau1[:, kk] = deltau
        u1[:, kk] = u
        # xm_old = xm.copy()
        if u >= 0:
            xm = A1@xm + B1@u
        elif u <0:
            xm = A2@xm + B2@u
        y = C@xm
        # xf = (Ae - Be@Kmpc) @ xf + Be @ Ke @ yr #xf = Ae @ xf + Be @ deltau
        # y = Ce @ xf

        y1[:,kk] = y.reshape(3,)
        # xf = np.vstack((xm - xm_old, y - yr)) #Xf = np.vstack((xm - xm_old, y - sp[:, kk + 1]))

    k = np.arange(N_sim)

    return u1, y1, deltau1, k

## initialization
Ts = 0.05
TIME_STEP = int(1000*Ts)
supervisor = Supervisor()
receiver = supervisor.getDevice("receiver")
emitter = supervisor.getDevice("emitter")
receiver.enable(TIME_STEP)

accel = 0
brake = 0

T_eng =  0.460 #0.26
K_eng = 0.732  # can add second order term if error is more
T_brake = 0.193
K_brake = 0.973
A_f = -1/T_eng
B_f = K_eng/T_eng
A_b = -1/T_brake
B_b = K_brake/T_brake

C = np.eye(3)
T_hw = 3
Ts = 0.05
T_total = 20
T = int(T_total/Ts)

#Discretize the system
A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_f]])
B = np.array([[0], [0], [B_f]])
sys2 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A1, B1, C, D , dt = sys2

A = np.array([[0, 1, -T_hw], [0, 0, -1], [0, 0, A_b]])
B = np.array([[0], [0], [B_b]])
sys3 = cont2discrete((A,B, C, 0), Ts, method='zoh')
A2, B2, C, D , dt = sys3


m1,n1 = C.shape  # m1 = 3, n1 = 3
n1,n_in = B.shape  # n_in = 1

###################
# Tuning parameters
#
a1 = 0.7 #0.6 and weight 10*Ce@Ce and R - 1, a = 5 - 7  ( 7 is slow, 5 is fast response)
N1 = 6
Np = 30
alpha = 1.03 ## Exponential weight
R1 = 1.2*np.eye(1,1)
R2 = 1.3*np.eye(1,1)
Q = np.transpose(C)@C
#
#
###################


_, S1, _ = cm.dlqr(A1, B1, Q, R1)
_, S2, _ = cm.dlqr(A2, B2, Q, R2)

Q1 = (1/alpha)**2*Q + (1-(1/alpha)**2)*S1
Q2 = (1/alpha)**2*Q + (1-(1/alpha)**2)*S2
R1 = (1/alpha)**2 * R1
R2 = (1/alpha)**2 * R2

a = [a1]
N = [N1]


# Initialize variables
N_sim = 50
Nc = 15

u = np.zeros((n_in, 1))
xm = np.array([[50], [10], [2]])
y = xm.copy()
yr = np.array([[0], [0], [0]])
M1, Lzerot = Mdu(a, N,Nc,n_in)

M0 = Mu(a, N,Nc,n_in)
_, Lz = lagd(a[0], N[0])
I = exp_matrix(N[0], alpha)

Omega1, Psi1 = dmpc(A1/alpha, B1/alpha, a, N, Np, Q1, R1)
Omega2, Psi2 = dmpc(A2/alpha, B2/alpha, a, N, Np, Q2, R2)

message = ''

y1 = np.array([[50], [10], [2]])
path = [[-100]]
failure_iter  = 0

while supervisor.step(TIME_STEP) != -1:
    # recive the data
    # print("this is sup Q length",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        x0 = rnd(np.frombuffer(message, dtype=np.float64))
        # print(f"this is x0 from supervisor  {x0} and shape {x0.shape} ")
        x0 = x0.reshape(3,1)
        receiver.nextPacket()

        path , y , deltau1 , k=simCon3(xm,u,y,yr,A1/alpha,A2/alpha,B1/alpha,B2/alpha,C,N_sim,Omega1, Omega2,Psi1, Psi2,Lz,M0, M1,I)

        y1 = y[:,0].reshape(3,1)
        # print(f"this is path {path}")

        cmd = []
        if path[0][0]!= -100 and len(path[0])>1:
            i=0
            for point in path[0]:
                cmd.append(point)
                i+=1
                if i>5:
                    break
        else:
            cmd = -100
            failure_iter += 1
            print(f"this is failure iter {failure_iter}")


        command = np.array(cmd)
        message = command.tobytes()

    if message != '' : #and message != previous_message:
        # previous_message = message
        emitter.send(message)
