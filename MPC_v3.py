from cvxpy import *
import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings('ignore')
from scipy.signal import cont2discrete , tf2ss
import control as ct
import mplcursors

DEBUG_ = False
import cProfile


def cvxpy_reverse2(A, B, N, Q, R, P, x0, xr, umax=None, umin=None, xmin=None, xmax=None):  
    is_sol =  False
    (nx, nu) = B.shape

    # mpc calculation  x0 = distance, x1 = velocity, x2 = accel 
    x = cvxpy.Variable((nx, N + 1))
    u = cvxpy.Variable((nu, N))
    weights = np.ones(8)
    w0 = weights[0]*0.1
    w1 = weights[1]*0.5
    w2 = weights[2]*0.15
    w3 = weights[3]*0.5
    w4 = weights[4]*0.05
    k1 = weights[5]*0.05
    k2 = weights[6]*0.2
    k3 = weights[7]*0.05
    # R = np.eye(nu)

    costlist = 0.0
    constraints = []

    for t in range(N):
        
        costlist +=  cvxpy.quad_form(x[:, t], Q) 
        # + k1*((x[0, t]-xr[0])*(x[1, t]-xr[1])) + k3*((x[2, t]-xr[2])*(x[0, t]-xr[0]))  + k2*((x[1, t]-xr[1])*(x[2, t]-xr[2]))
        # costlist += (w1*((x[1, t]-xr[1])) + w2*(x[2, t]-xr[2]) + w0*((x[0, t]-xr[0])))
        costlist += cvxpy.quad_form(u[:, t], R)
        # costlist += w3*(u[:, t] *u[:, t])

        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t]]

        if xmin is not None:
            constraints += [x[0, t] >= xmin[0, 0]]  # state is greater than x min
             
            # constraints += [xr[1] - x[1,t]  >= xmin[1, 0]]
            constraints += [x[2, t] >= xmin[1, 0]]
        if xmax is not None:
            constraints += [x[2, t] <= xmax[0]] # state is less than x max 
        if umax is not None:
            constraints += [u[:, t] <= umax]  # input constraints
        if umin is not None:
            constraints += [u[:, t] >= umin]  # input constraints

    # costlist += 0.5 *((x[1, N]-xr[1]) + (x[2, N]-xr[2]) + (x[0, N]-xr[0])) # terminal cost #0.5 *((x[1, N]-xr[1])**2 + (x[2, N]-xr[2])**2 + (x[0, N]-xr[0])**2)  # terminal cost
    # costlist += w4 *((x[1, N]) + (x[2, N])**2 + (x[0, N]))
    costlist +=  cvxpy.quad_form(x[:, N], P)

    constraints += [x[:, 0] == x0]  # inital state constraints
    
    ur1_max = 2
    ur1_min = -2
    # add input rate constraints
    # rate of change of u1 constraint
    # for i in range(N-1):
    #     constraints += [ (u[0,i+1] - u[0,i]) <= ur1_max]
    #     constraints += [ (u[0,i+1] - u[0,i]) >= ur1_min]

    # print("this is inside functio",x0)
    
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
    
    return is_sol , x.value, u.value[:,0], costlist.value

def reverse():
    print("start!! reverse ")
    T_eng =  0.460 #0.26  #
    K_eng = 0.732
    A_f = -1/T_eng
    B_f = K_eng/T_eng
    C = np.eye(3)
    T_hw = 3
    Ts = 0.05
    T_total = 30
    T = int(T_total/Ts)


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

    umax = 4
    umin = -4 

    xmin = np.array([[0] ,[-3.5]])  # state constraints
    xmax = np.array([2.5])  # state constraints

    x0 = np.array([40, 2.0, 0])  # initial state

    init_dist = x0[0] # initial distance between vehicles
    v0 = 5 # initial velocity of the reference vehicle
    
    x_current = reference3(T_total, Ts,init_dist , v0) # reference trajectory
    
    N = 20  # horizon
    iter = T #T # number of iteration
    x_hist = [x0]
    u_hist = [0]
    cost_hist = []
    xh = x_current[0,0] - x0[0]  # initial position of the host
    vh = x_current[0,1] - x0[1]  # initial velocity of the host
    d0 = 2.5 # tailgating distance between vehicles
    host_d = np.zeros((iter, 1))
    host_v = np.zeros((iter, 1))
    b = 0

    for i in range(iter):  #iter
        x_r = x_current[i, :] # is reference vehicle parameters
        d_ref = x_r[0] - (xh + T_hw*vh + d0)
        v_ref = x_r[1] - vh   # current values  

        x0 = np.array([d_ref, v_ref, x0[2]]) # referance state
        xr = np.array([0, 0 , 0]) # referance state
        try:
            notfail, x, u, c = cvxpy_reverse2(A, B, N, Q, R, P, x0, xr, umax = umax, umin = umin, xmin = xmin, xmax = xmax)
        except:
            u = np.array([-2.5]) #  u_hist[-1] #
            c = -1 
            b += 1
            print('No solution',b)
        
        x0 = A.dot(x0) + B.dot(u) #+ np.ranfdom.multivariate_normal(np.zeros(3), W)
        try:
            if len(x0[0]) >1:
                x0 = x0[0]
        except:
            pass
        
        print("this is car's states",x0,u)
        u_hist.append(u)
        print('Iteration:', i, 'Cost:', c, 'Input:', u)
        x_hist.append(x0)
        vh = vh + x0[2]*Ts
        xh = xh + vh*Ts + 0.5*x0[2]*Ts**2 
          
        host_d[i] = xh
        host_v[i] =  vh
        cost_hist.append(c)

    
    #################
    ##Plotings only
    ##below 
    #################
   
    x_hist = np.array(x_hist)
    u_hist = np.array(u_hist)
    cost_hist = np.array(cost_hist)
    u_limit = np.ones((iter, 2))
    u_limit[:,0] = umax
    u_limit[:,1] = umin

    acceleration  = np.diff(host_v[1:-1], axis=0)/Ts
    velocity = np.diff(host_d[1:-1], axis=0)/Ts
    
    # diff = x_current[0:iter+1,0] - x_hist[0:iter+1,0] 

    print("aa")
    plt.figure(figsize=(16, 32))    
    plt.subplot(5, 1, 1)
    plt.plot(host_d, label='host position', linewidth=2.5)
    plt.plot(x_current[0:iter,0], label='reference position', linewidth=2.5)
    plt.xlabel('Time/0.05' , fontsize=16)
    plt.xticks(fontsize=12)
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 2)
    plt.plot(host_v, label='host velocity', linewidth=2.5)
    plt.plot(x_current[0:iter,1], label='reference velocity', linewidth=2.5)
    # plt.plot(velocity, label='velocity', linewidth=2.5, linestyle = '-.')
    plt.xlabel('Time/0.05' , fontsize=16)
    plt.xticks(fontsize=12)
    plt.legend()
    plt.grid(True)

    plt.subplot(5, 1, 3)
    plt.plot(x_hist[0:iter,2], label='host accel', linewidth=2.5)
    plt.plot(x_current[0:iter,2], label='reference accel', linewidth=2.5)
    plt.plot(u_hist, label='input', linewidth=2.5)
    
    plt.xlabel('Time/0.05' , fontsize=16)
    plt.xticks(fontsize=12)
    plt.grid(True)
    plt.legend()

    plt.subplot(5, 1, 4)
    plt.plot(x_hist[:,0], label='state del d', linewidth=2.5)
    plt.xlabel('Time/0.05' , fontsize=12)
    plt.grid(True)
    plt.legend()

    plt.subplot(5, 1, 5)
    plt.plot(x_hist[:,1], label='state del v', linewidth=2.5)
    plt.xlabel('Time/0.05' , fontsize=12)
    plt.grid(True)
    plt.legend()


    plt.show()
    
    plt.plot(acceleration, label='acceleration', linewidth=2.5)
    plt.show()
    
    # plotting(x_hist,u_hist,cost_hist,host_d,x_current[:,0], host_d,True)
    
    return x_hist #cost_hist


if __name__ == '__main__':
    DEBUG_ = False
    Ts = 0.05
    #x_rev = cProfile.run('reverse()')
    x_rev = reverse()
    # print(x_rev)  