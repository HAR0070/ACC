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
states are maintaned as dictionory
each state has a id
order is x v a parent_id cost
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

def reverse_propogation(u,state_p, plot = False):
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
        x[i+1] = x[i] - x_dot*dt - x_doubledot*dt
        x_dot -= x_doubledot * dt
        x_doubledot = (x_doubledot - (k * u)*dt/T )/0.9
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
    diff2 = (goal[1] - neighbor[1])**2
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
    s_fringe = States()
    g_fringe = States()
    node = [*start, parent, 0 , 0]
    s_fringe.add(id, node)
    g_fringe.add(id,node)
    sc_fringe = States()
    gs_fringe = States()

    path = []
    y = 0
    if is_goal(start, goal, do, T_hw, resolution):
        return path

    b = 0
    jerk = [i * 0.3 for i in range(-5, 5)]
    while s_fringe.len() >0 or g_fringe.len():
        # print("len of the fringe is" + str(Ofringe.len()))
        # print(f"this is the open fringe {Ofringe}")
        b +=1
        if s_fringe.len() >0:           """ eddited till here for biderection"""
        key, node = Ofringe.remove()
        # print("a node is popped", node)
        # print(f"cost of poped node is {node[4]}")
        parent_id = key
        Cfringe.add(key, node)

        for u in jerk:
            state_p = node # copy of the parent node

            if abs(state_p[2] - u) > 1.5:
                continue

            trajectory, ds , acel , v = reverse_propogation(u, state_p) # add false
            neighbor = [trajectory[-1][0] , v, acel]
            # print(f"is the x correct {trajectory[-1][0]} and {trajectory}")
            # print("this is the neighbor", neighbor)

            if acel > 2 or acel < -3.5:
                continue

            if ds < 0 or v<0:
                continue

            if neighbor[0] > goal[0]:
                continue

            if is_goal(neighbor, goal, do, T_hw, resolution):
                # print("checking is_goal")
                for id in range(0,id):
                    try:
                        plt.scatter(Ofringe.x(id), Ofringe.v(id), marker='x')
                    except:
                        pass
                path.append(neighbor[0:3])
                # plt.scatter(i,i)
                for id in reversed(state_p[3]):
                    path.append([Cfringe.x(id), Cfringe.v(id), Cfringe.a(id)])
                    # plt.scatter(Cfringe.input(id), Cfringe.input(id),marker='#')
                return path


            ## cost
            # h1 = (goal[0] + T_hw*v + do) - start[0]
            # h2 = 0.1*(start[1] - goal[1])**2 +  (start[2] - goal[2])**2
            h1 = 0
            h2 = 0
            # cost = (0.1/(ds + 0.1)) + 2*jerk**2 + 5*a**2
            cost1 = (goal[0]- neighbor[0])**2 # + do
            cost2 = (acel)**2  #+ 1/(ds + 0.1) + 2*jerk**2
            cost3 = (goal[1] - neighbor[1])**2
            # h2 = -(neighbor[1] - goal[1]) +  -(neighbor[2] - goal[2])
            # print(f"huristics are {cost1} and {cost2}")

            # print(f"the costs are jerk {2*jerk**2} , {2*(a-state_p[2])**2} , if ds was there {0.1/(ds + 0.1)}")
            # print(f"this is h1 {h1} and this is h2 {h2} the cost is {cost} ")
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
                        # plt.scatter(i,i)
                        # print("appending through check", nodea)
                        # plt.scatter(nodea[0],nodea[1], marker='+', s =20)
                        # plt.show()
                        # plt.pause(0.001)
                        break
                    else:
                        break
                else:
                    continue
            if tag:
                id+=1
                nodeb = [*neighbor, [*node[3], parent_id], cost+ node[4], u]
                Ofringe.add(id,nodeb)
                # plt.scatter(i,i)
                # y += 1
                # plt.scatter(nodeb[0],nodeb[1],marker='o', s= 10)
                # plt.show()
                # plt.pause(0.001)
                # print("appending directly", nodeb)


    return 'brake'


if __name__=='__main__':
    # start = x , v, a
    # goal = x, v, a
    do = 0
    T_hw = 1.5
    resolution = np.sqrt(0.01)
    intial = 0 , 0 , 0
    goal = 2 , 0 , 0
    # fig =  plt.figure()
    # fig.suptitle('fringe items')
    # plt.xlabel('position')
    # plt.ylabel('velocity')
    # plt.title('Scatter Plot')
    path = astar_v(intial, goal,do, T_hw, resolution)
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
