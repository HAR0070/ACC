import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit , minimize_scalar
from scipy.spatial.distance import euclidean
import math
import time


plt.ion()

"""
states are maintaned as dictionory
each state has a id
order is x y theta delta Sp parent_id cost
"""
class States:
    def __init__(self):
        self.data = {}

    def add(self, id, value):
        if len(value) !=7:
            raise ValueError("Values must be a list of 7 elements")
        self.data[id] = value

    def get(self, id):
        return self.data.get(id)

    def xy(self, id):
        return self.data.get(id)[0:2]

    def sp(self, id):
        return self.data.get(id)[4]

    def theta(self, id):
        return self.data.get(id)[2]

    def delta(self,id):
        return self.data.get(id)[3]

    def parent_id(self,id):
        return self.data.get(id)[5]

    def cost(self,id):
        return self.data.get(id)[6]

    def len(self):
        return len(self.data)

    def key(self):
        return self.data.keys()

    def remove(self):
        self.data = dict(sorted(self.data.items(), key=lambda x: x[1][6]))
        key = next(iter(self.data))
        value = self.data.pop(key)
        return key, value

# delta = steering angle
# given state_p = state.get(id)[0:4]
def bicycle_model(state_p, del_d, v, L, dt):
    # print("now bicycle model")
    x, y, theta, delta = state_p
    delta += del_d
    beta = np.arctan(np.tan(delta)/2)
    x = x + v* np.cos(theta + beta)*dt
    y = y+v*np.sin(theta + beta)*dt
    theta = theta + v*np.sin(beta)*dt/L
    return [x,y,theta,delta]

# trajectory generation for non varying theta
# give state = state.get(id)
def traj_1(state, del_d, T, PLOT= True):
    # print("now traj_1")

    trajectory = [state[0:4]]
    ds = 0
    for i in range(int(T/dt)):
        state_p = trajectory[-1]
        x1,y1 = state_p[0:2]
        x,y,theta,delta = bicycle_model(state_p,del_d, v, L, dt)
        ds += np.sqrt((x-x1)**2 + (y - y1)**2)
        trajectory.append([round(x,3),round(y,3),theta,delta])

    trajectory = np.array(trajectory)
    if PLOT == True:
        plt.plot(trajectory[:,0], trajectory[:,1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Trajectory of Kinematic Bicycle Model')
        plt.show()
        plt.pause(0.01)
    return trajectory , ds

###########################
# tracking the vehicle along the centre line
# state as state only
def track_pos(neighbor, sp, coord, ds):
    x,y = neighbor[0:2]
    s = sp + ds
    # which is the nearest s in coord
    close_s = np.argmin(np.abs(coord[3] - s))
    dist = []
    n = 4
    for i in range(-n,n):
        xt , yt = coord[0][close_s+i] , coord[1][close_s+i]
        dist.append(np.sqrt((xt-x)**2 + (yt - y)**2))
    dist = np.array(dist)

    closest_s = np.argmin(dist)
    yerr = dist[closest_s]
    pos = close_s + closest_s - n
    theta_err =  np.abs(coord[2][pos] - neighbor[2])
    Sp = coord[3][pos]

    return yerr, theta_err, Sp

#############################
# checking if we reached

def is_goal(sp, neighbor_sp, d , coord):
    # print("is_goal")
    s = sp + d

    closest_s = np.argmin(np.abs(coord[3] - s ))
    plt.scatter(coord[0][closest_s] , coord[1][closest_s], s =100)

    nclose = np.argmin(np.abs(coord[3] - neighbor_sp ))
    plt.scatter(coord[0][nclose] , coord[1][nclose], s =50)
    if neighbor_sp > sp + d:
        return True
    else:
        return False

###########################
# currently nothing is there
def check_obstacle(neighbor , obstacle ):
    ####
    # call collision checker in webots
    # True if the point in obstacle
    ####
    tag = False
    x , y = obstacle
    plt.scatter(x,y,s=200)
    diff = (neighbor[0] - x)**2 + (neighbor[1] - y)**2
    if ( diff < resolution**2):
        tag = True
    return tag

# Define the A* algorithm function
# change to coord
def astar(start, resolution , coord):
    print("now running A*")
    parent = []
    states = States()
    start_s = start[4]
    id = 0
    d = v*T
    node = [*start[0:4],0, parent, 0]
    states.add(id,node)
    closed_set = States()
    path = []

    # print(" printing values used for goal", start_s ,v,T )
    if is_goal(start_s,0,d, coord):
        return "got path"

    b = 0
    while states.len() >0 :
        # score_ = []
        b += 1
        # print("this is len of open set before" , states.len())
        key, node = states.remove()  # key will be the parent id
        print(" the poped node is", node)
        parent_id = key
        closed_set.add(key , node)
        path = []

        for delta_ in range(-40,40,5):
            dd = np.deg2rad(delta_)
            state_p = node
            print("this is printing id", id)

            trajectory , ds = traj_1(state_p, dd , Th, PLOT=False)
            neighbor = trajectory[-1]  #as 2 pieces so as to plot the trajectory later
            yerr, theta_err, state_p[4] = track_pos(neighbor, node[4] , coord, ds)


            # current nodes Sp is updated here
            # print(" this is yerr", yerr)
            if abs(yerr) > 3.5:
                continue

            ## need to change this into current sp - prev sp < 0
            if (state_p[4] - node[4]) < 0:
                continue

            obstacle = [10,40]
            if check_obstacle(neighbor, obstacle):
                continue

            # print("neighbor sp is ",state_p[4])
            if is_goal(start_s ,state_p[4], d, coord):
                for id in state_p[5]:
                    path.append([*closed_set.xy(id),closed_set.theta(id)])
                return path

            h = (start_s + v*T - state_p[4])**2  # how fas is the goal distance
            cost1 = ds**2   # ds is path length of trajectory
            cost2 = yerr**2   #+ theta_err**2  # cross track error and theta error
            # theta in coord is becoming nan sometime, and hence cost is becoming nan
            state_p[6] = round((node[6]+ cost1 + h + cost2) , 3) # cost is updated


            a = True # current neighbor is open to be added to states
            for key in states.key():
                x , y = states.xy(key)
                diff = (neighbor[0] - x)**2 + (neighbor[1] - y)**2
                if ( diff < resolution**2):
                    # print("this is the difference and key", diff, "  ", key)
                    a = False # the state no longer needs to be checked to be added or not
                    if state_p[6] < states.cost(key):
                        id+= 1
                        if len(state_p[5]) >0:
                            if state_p[5][-1]!= parent_id :
                                state_p[5].append(parent_id)
                        else:
                            state_p[5].append(parent_id)
                        node = [ *neighbor,state_p[4], state_p[5],state_p[6] ]
                        states.add(id,node) # the neighbour is added to state

                        plt.plot(trajectory[:,0], trajectory[:,1],color='#FF0000')
                        plt.show()
                        plt.pause(0.01)
                        # print(" appending neighbour throug a")
                        break
                    else: # print("rejected")
                        break
                else:
                    continue

            if a:  # if there is no node within the resolution in states
                id+= 1
                if len(state_p[5]) >0:
                    if state_p[5][-1]!= parent_id :
                        state_p[5].append(parent_id)
                else:
                    state_p[5].append(parent_id)
                node = [*neighbor,state_p[4], state_p[5],state_p[6]]
                states.add(id,node) # the neighbour is added to state

                plt.plot(trajectory[:,0], trajectory[:,1],color='#FF0000')
                plt.show()
                plt.pause(0.01)
                # print(" appending neighbour throug b")
        #print(score_)

    return 'brake'

#########################
# converting track to x,y,ds and theta - heading angle
def track(n):
    df = pd.read_csv('gps_data2.csv')
    df[['x','y']] = df[['x','y']].astype(float)
    arr = df.to_numpy()
    x = []
    y = []

      ## moving average
    for i in range(int(len(arr)/n)):           #
        x.append(sum(arr[i*n:(i+1)*n, 0])/n)
        y.append(sum(arr[i*n:(i+1)*n ,1])/n)

    x0 = x[1]
    y0 = y[1]
    x[0] = x[1]     # replacing nan
    y[0] = y[1]

    x = np.array(x)
    y = np.array(y)
    x -=x0
    y -=y0

    coord = np.array(x[2:])
    coord = np.vstack([coord , y[2:]])
    ## finding radius of curvature

    dx = x[2:] - x[:-2]
    dy = y[2:] - y[:-2]
    d2x = x[2:] - 2*x[1:-1] + x[:-2]
    d2y = y[2:] - 2*y[1:-1] + y[:-2]

    theta = np.abs(dx*d2y - dy*d2x) / (dx**2 + dy**2)**(3/2) # radius of  curvature
    coord = np.vstack([coord , theta])

    dist = [0] * (len(x) - 1)
    for i in range(len(x)-1):
        dist[i] = np.sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2)
    del dist[0:2]
    dist = [0] + np.cumsum(dist).tolist()
    coord = np.vstack([coord, dist])

    return coord

#########################
# input parameters

v = 6 #velocity
L = 2.5 #  wheelBase
dt = 0.5 # time step
# x0, y0, theta0,sp , delta =   0, 0, np.deg2rad(0), np.deg2rad(0), 0

Th = 2
resolution = np.sqrt(3)
T = 10 # time horizon for planning in front of car

if __name__=='__main__':
    n = 4
    coord = track(n)
    x0, y0, theta0 , delta, sp =   0, 0, np.deg2rad(120), np.deg2rad(0), 0

    # # delta = 0
    start_time = time.time()

    present = x0, y0 , theta0, delta, sp

    path = astar(present , resolution, coord)
    if path != 'brake':
        xp = path[0]
        yp = path[1]
        plt.scatter(xp,yp)
        end_time = time.time()
        print("!!!!!!!!!!!!!!!!!!!!!!##############################!!!!!!!!!!!!!!!!!!!!!")

        print(end_time - start_time)
        # x = coord[0]
        # y = coord[1]
        # plt.scatter(x,y)
        plt.show()
        plt.pause(10)
    else:
        print('brake')

"""
path is key of dict -  the closed_set
its better to get x , y theta coordinates there itself
"""
