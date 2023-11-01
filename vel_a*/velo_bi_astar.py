    # Astar for velocity conrtol - longitudanal motion control - ACC

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
# from scipy.optimize import curve_fit , minimize_scalar
# from scipy.spatial.distance import euclidean
import math
import time

plt.ion()
plt.axis('equal')
"""
Needs work on
    - Resolution for removing same nodes
    - Resolution for is_goal
    - Cost function
"""
"""
Give 10 inputs as results - if by next step solution is not found use these
Change vehicle model to suit webots
Make change in webots to include 1st order Dynamics
Find the time step with which driver.c works with
"""


class States:
    """
    states are maintaned as dictionory
    each state has a id
    order is x v a parent_id cost
    """
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

def forward_propogation(u,state_p, plot = False):
    # Initialization
    step_t = 1.5
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
    iter = 0
    for i in range(N):
        # Compute the jerk (third derivative of x) using the equation
        # Update the position, velocity, and acceleration using Euler's method
        x[i+1] = x[i] + x_dot*dt + x_doubledot*dt
        x_dot += x_doubledot * dt
        if x_doubledot < 2.5 or x_doubledot > -3.5:
            x_doubledot = x_doubledot*0.98  + k*u*(1 - alpha)
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        elif x_doubledot > 2.5:
            x_doubledot  = 2.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        elif x_doubledot < -3.5:
            x_doubledot = -3.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        # Store or use the generated state x
        t[i+1] = t[i] + dt
    if plot:
        plt.scatter(x,t)
        plt.show()
        plt.pause(0.001)
    return x , x[-1]-x[0], accel_to_cmd, x_dot

def reverse_propogation(u,state_p, plot = False):
    # Initialization
    step_t = 1.5
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
    iter = 0
    for i in range(N):
        # Compute the jerk (third derivative of x) using the equation
        # Update the position, velocity, and acceleration using Euler's method
        x[i+1] = x[i] - x_dot*dt - 0.5*x_doubledot*dt*dt
        x_dot -= x_doubledot * dt
        if x_doubledot <= 2.5 or x_doubledot >= -3.5:
            x_doubledot = (x_doubledot - k*u*(1 - alpha))/0.98
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        elif x_doubledot > 2.5:
            x_doubledot  = 2.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        elif x_doubledot < -3.5:
            x_doubledot = -3.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter+=1
        # Store or use the generated state x
        t[i+1] = t[i] + dt
    if plot:
        plt.scatter(x,t)
        plt.show()
        plt.pause(0.001)
    return x , x[0]-x[-1], accel_to_cmd, x_dot

def is_goal(neighbor, goal, do, T_hw, res):
    # goal will be x coordinate with a and v
    diff1 = (goal[0] - neighbor[0])**2
    diff2 = (goal[1] - neighbor[1])**2
    diff3 = (goal[2] - neighbor[2])**2
    res = 0.2
    if (diff3 < res) and (diff2 < res) and (diff1 < res):
        return True

def solution_path(gkey,skey,cg_fringe,cs_fringe):
    print("Path found by intersection")
    path = []
    for id in cs_fringe.parent_id(skey):
        path.append([cs_fringe.x(id), cs_fringe.v(id), cs_fringe.a(id), cs_fringe.input(id)])
    for id in reversed(cg_fringe.parent_id(gkey)):
        path.append([cg_fringe.x(id), cg_fringe.v(id), cg_fringe.a(id), cg_fringe.input(id)])
    return path

def astar_v(start, goal, do, T_hw, resolution):
    # print("now running a star")
    parent = []
    id = 0
    s_fringe = States()   # open fringe
    g_fringe = States()
    nods = [*start, parent, 0 , 0]
    s_fringe.add(id, nods)
    nodg = [*goal, parent, 0,0]
    g_fringe.add(id,nodg)
    cs_fringe = States() # closed fringe
    cg_fringe = States()

    path = []  # the return path
    y = 0
    if is_goal(start, goal, do, T_hw, resolution):
        return 'brake'

    jerk = [i * 0.3 for i in range(-5, 5)]
    res = resolution**2

    while s_fringe.len() >0 or g_fringe.len() >0:

        if s_fringe.len() >0:
            # print("opening s fringe")
            key, nods = s_fringe.remove()
            parent_id = key
            cs_fringe.add(key, nods)

            if is_goal(nods[0:3], goal, do, T_hw, resolution):
                # print("checking is_goal")
                for id in range(0,id):
                    try:
                        plt.scatter(s_fringe.x(id), s_fringe.v(id), marker='x')
                    except:
                        pass
                for id in reversed(cs_fringe.parent_id(parent_id)):
                    path.append([cs_fringe.x(id), cs_fringe.v(id), cs_fringe.a(id), cs_fringe.input(id)])
                return path

            for key in cg_fringe.key():
                diff1 = (nods[0] - cg_fringe.x(key))**2
                diff2 = (nods[1] - cg_fringe.v(key))**2
                diff3 = (nods[2] - cg_fringe.a(key))**2
                if (diff3 < res) and (diff2 < res) and (diff1 < res):
                    print("path found s intersecting g")
                    return solution_path(key,parent_id,cg_fringe,cs_fringe)
                else:
                    continue

            for u in jerk:
                # print(" making sub nods")
                state_p = nods # copy of the parent node

                if abs(state_p[5] - u) > 1:
                    continue

                trajectory, ds , acel , v = forward_propogation(u, state_p) # add false
                neighbor = [trajectory[-1][0] , v, acel]

                if acel > 2 or acel < -3.5:
                    continue

                if ds < 0 or v<0:
                    continue

                if neighbor[0] > goal[0]:
                    continue

                cost = 0.2*u**2 + (cs_fringe.v(parent_id) - neighbor[1])**2 + (cs_fringe.a(parent_id) - acel)**2 + (cs_fringe.v(parent_id)*1.5 - ds)**2
                h2 = 3*(goal[0] - neighbor[0])**2  + acel**2
                cost = h2 + cost

                tag = True
                for key in s_fringe.key():
                    diff1 = (neighbor[0] - s_fringe.x(key))**2
                    diff2 = (neighbor[1] - s_fringe.v(key))**2
                    diff3 = (neighbor[2] - s_fringe.a(key))**2
                    if (diff3 < res) and (diff2 < res) and (diff1 < res):
                        # print("node comes inside resolution")
                        tag = False
                        if state_p[4] < s_fringe.cost(key):
                            id+=1
                            nodea = [*neighbor, [*nods[3], parent_id], cost+ nods[4], u]
                            s_fringe.add(id,nodea)
                            s_fringe.rmv_id(key)
                            break
                        else:
                            break
                    else:
                        continue
                if tag:
                    id+=1
                    nodeb = [*neighbor, [*nods[3], parent_id], cost+ nods[4], u]
                    s_fringe.add(id,nodeb)

        if g_fringe.len() >0:
            # print("Opening g fringe")
            key, nodg = g_fringe.remove()
            parent_id = key
            cg_fringe.add(key, nodg)

            if is_goal(nodg[0:3], start, do, T_hw, resolution):
                # print("checking is_goal")
                for id in range(0,id):
                    try:
                        plt.scatter(g_fringe.x(id), g_fringe.v(id), marker='x')
                    except:
                        pass
                for id in cg_fringe.parent_id(parent_id):
                    path.append([cg_fringe.x(id), cg_fringe.v(id), cg_fringe.a(id), cg_fringe.input(id)])
                return path

            for key in cs_fringe.key():
                diff1 = (nodg[0] - cs_fringe.x(key))**2
                diff2 = (nodg[1] - cs_fringe.v(key))**2
                diff3 = (nodg[2] - cs_fringe.a(key))**2
                if (diff3 < res) and (diff2 < res) and (diff1 < res):
                    print("path found g intersecting s")
                    return solution_path(parent_id,key,cg_fringe,cs_fringe)
                else:
                    continue

            for u in jerk:
                state_p = nodg # copy of the parent node

                if abs(state_p[5] - u) > 1.5:
                    continue

                trajectory, ds , acel , v = reverse_propogation(u, state_p) # add false
                neighbor = [trajectory[-1][0] , v, acel]

                if acel > 2 or acel < -3.5:
                    # print("nodg droppping here for acel")
                    continue

                if ds < 0 or v<0:
                    # print("nodg dropping here for ds and v")
                    continue

                if neighbor[0] < start[0]:
                    continue

                # print("producing nodg")
                cost = 0.2*u**2 + (cg_fringe.v(parent_id) - neighbor[1])**2 + (cg_fringe.a(parent_id) - acel)**2 + (cg_fringe.v(parent_id)*1.5 - ds)**2
                h2 = 3*(start[0] - neighbor[0])**2  + acel**2
                cost = h2 + cost

                tag = True
                for key in g_fringe.key():
                    diff1 = (neighbor[0] - g_fringe.x(key))**2
                    diff2 = (neighbor[1] - g_fringe.v(key))**2
                    diff3 = (neighbor[2] - g_fringe.a(key))**2
                    if (diff3 < res) and (diff2 < res) and (diff1 < res):
                        # print("node comes inside resolution")
                        tag = False
                        if state_p[4] < g_fringe.cost(key):
                            id+=1
                            nodea = [*neighbor, [*nodg[3], parent_id], cost+ nodg[4], u]
                            g_fringe.add(id,nodea)
                            g_fringe.rmv_id(key)
                            break
                        else:
                            break
                    else:
                        continue
                if tag:
                    id+=1
                    nodeb = [*neighbor, [*nodg[3], parent_id], cost+ nodg[4], u]
                    g_fringe.add(id,nodeb)

    return 'brake'


if __name__=='__main__':
    # start = x , v, a
    # goal = x, v, a
    prev_time = time.time()
    do = 0
    T_hw = 1
    resolution = np.sqrt(0.01)
    intial = 45 , 0 , 0
    goal = 85 , 0 , 0
    dt = 0.1
    T = 0.532
    alpha = math.exp(-dt / T)

    path = astar_v(intial, goal,do, T_hw, resolution)
    curr_time = time.time()
    print(f"this is the time taken {curr_time-prev_time}")
    if path != 'brake':
        print("!!!!!!!!!!!!!!!!!!!!!!###########################!!!!!!!!!!!!!!!!!!!!!!!!!!")
        path = np.array(path)
        x = path[:,0]
        v = path[:,1]
        a = path[:,2]
        plt.plot(x,v, label = "position" ,linewidth=2.5)
        plt.scatter(goal[0], goal[1] , marker='D')
        # plt.plot(v,label = "velocity" , linewidth=2.5 )
        plt.plot(x,a ,label = "accel",  linewidth=2.5)

        plt.legend()
        plt.show()
        plt.pause(60)
    else:
        print('brake')
        plt.pause(30)


    ## to plot we should actually take the jerk at all 0.1 time step and execute
    ## so need to store the first parent-nodes jerk inputs
    ## modify later
