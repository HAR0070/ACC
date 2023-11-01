"""host_Astar_supervisor controller."""

import numpy as np
import pandas as pd
# from scipy.optimize import curve_fit , minimize_scalar
# from scipy.spatial.distance import euclidean
import math
import time


from controller import Supervisor

## initialization
Ts = 0.05
TIME_STEP = int(1000*Ts)
supervisor = Supervisor()
receiver = supervisor.getDevice("receiver")
emitter = supervisor.getDevice("emitter")
receiver.enable(TIME_STEP)

accel = 0
brake = 0

previous_message = ''
xr = [0 , 0 , 0]

curr_time = 0
prev_time = 0


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

def rnd(number, precision=2):
    if isinstance(number, (int, float)):
        return round(number, precision)
    if isinstance(number , np.ndarray):
        return np.round(number, precision)

def propogation(u,state_p,step_t= 0.5):
    # Initialization
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
        x[i+1] = x[i] + x_dot*dt + 0.5*x_doubledot*dt*dt
        x_dot += x_doubledot * dt
        if x_doubledot < 2.5 or x_doubledot > -3.5:
            x_doubledot = x_doubledot*0.98  + k*u*(1 - alpha)
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter +=1
        elif x_doubledot > 2.5:
            x_doubledot  = 2.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter +=1
        elif x_doubledot < -3.5:
            x_doubledot = -3.5
            if iter ==0:
                accel_to_cmd = x_doubledot
                iter +=1
        # Store or use the generated state x
        t[i+1] = t[i] + dt
    return x , x[-1]-x[0], accel_to_cmd, x_dot

def is_goal(neighbor, goal, res):
    # goal will be x coordinate with a and v
    diff1 = (goal[0] - neighbor[0])**2
    diff2 = (goal[1] - neighbor[1])**2
    diff3 = (goal[2] - neighbor[2])**2
    res = 0.3
    ########
    #
    # correct the resolution later
    #
    #######
    if (diff3 < res/2) and (diff2 < res) and (diff1 < res*2):
        return True

def astar_v(start, goal,resolution,step_t):
    # print("now running a star")
    parent = []
    id = 0
    Ofringe = States()
    node = [*start, parent, 0 , 0]
    Ofringe.add(id, node)
    Cfringe = States()
    path = []
    if is_goal(start, goal,resolution):
        return path

    b = 0
    jerk = [i * 0.3 for i in range(-5, 5)]
    while Ofringe.len() >0:
        b +=1
        keyp, node = Ofringe.remove()
        parent_id = keyp
        Cfringe.add(keyp, node)

        for u in jerk:
            state_p = node.copy() # copy of the parent node

            if abs(state_p[5] - u) > 2:
                continue

            trajectory, ds , acel , v = propogation(u, state_p,step_t)
            neighbor = [trajectory[-1] , v, acel]

            if acel > 2.5 or acel < -3.5:
                continue

            if ds < 0 or v<0:
                continue

            if neighbor[0] > goal[0]:
                continue

            if is_goal(neighbor, goal, resolution):
                # plt.scatter(i,i)
                for id in state_p[3]:
                    path.append([Cfringe.x(id), Cfringe.v(id), Cfringe.a(id)])
                path.append(neighbor[0:3])
                return path
            # if is_goal(neighbor, goal, resolution):
            #     path.append([*neighbor[0:3],u])
            #     # plt.scatter(i,i)
            #     # command = Cfringe.input(state_p[3][0])
            #     for id in reversed(state_p[3]):
            #         path.append([Cfringe.x(id), Cfringe.v(id), Cfringe.a(id),Cfringe.input(id)])
            #         # plt.scatter(Cfringe.input(id), Cfringe.input(id),marker='#')
            #     return command

            cost = 0.2*u**2 + 0.5*(Cfringe.v(parent_id) - neighbor[1])**2 + 0.5*(Cfringe.a(parent_id) - acel)**2 + (Cfringe.v(parent_id)*1.5 - ds)**2
            h2 = (goal[0] - neighbor[0])**2  + acel**2

            cost = h2 + cost

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


print("the A* supervisor controller is running")

resolution = np.sqrt(0.01)
step_t = 1.5
dt = 0.1
T = 0.532
alpha = math.exp(-dt / T)
failure_iter = 0

while supervisor.step(TIME_STEP) != -1:

    # print("now the controller is running")
    # recive the data
    # print("this is sup Q length",receiver.getQueueLength())
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        msg = rnd(np.frombuffer(message, dtype=np.float64))
        goal = [msg[0],msg[1],0]
        initial= [msg[2],msg[3],msg[4]]
        print(f"this is x0 from vehicle controller {msg} ")
        receiver.nextPacket()

      # give to optimization
      # a is acceleration output from a*
        path = astar_v(initial, goal, resolution,step_t)
        if path != 'brake' and len(path) >= 2:
            a = path[1][-1]
            if  a >=0:
                brake = np.nan
                accel = a/2.5
            else:
                accel = np.nan
                brake = a/-3.5
            # else:
            #     a = path[0][-1]
            #     if  a >=0:
            #         brake = np.nan
            #         accel = a/2
            #     else:
            #         accel = np.nan
            #         brake = a/-3.5
        else:
            brake = 10
            accel = np.nan
            failure_iter += 1
            if step_t > 0.5 and failure_iter > 2: #(goal[0] - initial[0])< 15 and
                step_t = step_t - 0.1
                failure_iter = 1
            # elif (goal[0] - initial[0]) > 15 and step_t < 2 and a >3:
            #     step_t = step_t +0.2
            #     failure_iter = 1

    command = np.array([accel, brake])
    message = command.tobytes()
    # send the commands
    # message_size = sys.getsizeof(message)
    # print(message)

    if message != '' and message != previous_message:
        previous_message = message
        emitter.send(message)
        # print(f"sent message from the controller is {command} ")

    curr_time = time.time()
    print("time taken is", curr_time - prev_time)
    print(f"this is failure attempt {failure_iter}")
    prev_time = time.time()
