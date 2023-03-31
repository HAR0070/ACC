import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit , minimize_scalar
from scipy.spatial.distance import euclidean
from heapq import heappush, heappop
import math

# Kinematic Bicycle mode
############################33
# We will only execute first 2 steps of the a astar and recompute astar after that
#
###############



# delta = steering angle

def bicycle_model(state, delta, v, L, dt):
    x, y, theta = state
    beta = np.arctan(np.tan(delta)/2)
    x = x + v* np.cos(theta + beta)*dt
    y = y+v*np.sin(theta + beta)*dt
    theta = theta + v*np.sin(beta)*dt/L
    return [x,y,theta]

# trajectory generation for non varying theta
def traj_1(state, delta, T, PLOT= True):
    trajectory = [state]
    for i in range(int(T/dt)):
        state = trajectory[-1]
        x,y,theta = bicycle_model(state,delta, v, L, dt)
        #print(x,y,theta)
        trajectory.append([x,y,theta])

    trajectory = np.array(trajectory)
    # Plot the trajectory
    if PLOT == True:
        plt.plot(trajectory[:,0], trajectory[:,1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Trajectory of Kinematic Bicycle Model')
        plt.show()
    return trajectory[-1]

## take gps coordinates as input and do spline fitting
def func(x,a,c,g):
    return a*x**2 + c*x + g

def generate_centre_l(df):

    df.drop_duplicates()
    df.dropna(inplace=True)
    x = df['x']
    y = df['y']
    # popt has the optimized parameters
    popt, pcov = curve_fit(func, x, y)
    return  popt

#######################333
#
#########################
def cost(current, neighbor):
    def max_distance():
        trajectory = traj_1(current, 0, 0.5, False)
        # since it will be a straight line along x axis
        d = np.sqrt((current[0] - trajectory[0])**2 + (current[1] - trajectory[1])**2)
        if d > 0:
            return d
        else:
            return 10

    dx = abs(current[0] - neighbor[0])
    dy = abs(current[1] - neighbor[1])
    d = np.sqrt(dx**2 + dy**2)
    dtheta = abs(current[2] - neighbor[2])
    maxd = max_distance()
    max_theta = 10
    # nor normalize variables
    #norm_d = (d - 0)/(maxd - 0)
    #norm_theta = (dtheta - 0)/(10 - 0)
    cost = d #np.sqrt(norm_d**2 + norm_theta**2)
    print("this is cost", cost)
    return cost

def cost2(neighbor):
    x0 , y0 = neighbor[0:2]
    # def dist_square(x , x0, y0, popt):
    #     return ((func(x, *popt) - y0)**2 + (x - x0)**2)
    # we are finding the point on curve with minimum distance with x0 and y0
    # x1 = minimize_scalar(lambda x: dist_square(x, x0, y0)).x
    # x1 = np.min(x, key= lambda x: dist_square(x , x0, y0, popt))
    distance = []
    # for i in range(df.shape[0]):
    #     x , y = df.iloc[i,0:2]
    #     p1 = [x , y]
    #     p2 = [x0, y0]
    #     distance.append(euclidean (p1, p2))
    for i in range(df.shape[0]):
        x = df.iloc[i, 0]
        y = df.iloc[i,1]
        distance.append(np.sqrt((x-x0)**2 + (y - y0)**2))

    min_index = np.argmin(distance)
    x1 , y1 = df.iloc[min_index, 0:2]

    h = np.sqrt((x1 - x0)**2 + (y1-y0)**2)
    h = (h - 0)/(3 - 0)
    print("this is huristic", h)
    return h

def heuristic(neighbor, v, Th , start, df):
    x = neighbor[0]
    y = neighbor[1]
    xg , yg = set_goal(v, Th, start , df)
    return np.sqrt((x - xg)**2 + (y-yg)**2)

def set_goal(v, Th , start, df):
    Th = 2  ######################################################################################################
    d = Th*v
    x0 , y0 = start[0:2]
    # point on the curve nearest to current state
    distance = []
    # for i in range(df.shape[0]):
    #     x , y = df.iloc[i,0:2]
    #     p1 = [x , y]
    #     p2 = [x0, y0]
    #     distance.append(euclidean (p1, p2))
    for i in range(df.shape[0]):
        x = df.iloc[i, 0]
        y = df.iloc[i,1]
        distance.append(np.sqrt((x-x0)**2 + (y - y0)**2))

    min_index = np.argmin(distance)
    x1 , y1 = df.iloc[min_index, 0:2]
    # def dist_square(x , x0, y0):
    #     return ((func(x, *popt) - y0)**2 + (x - x0)**2)
    # # we are finding the point on curve with minimum distance with x0 and y0
    # x1 = minimize_scalar(lambda x: dist_square(x, x0, y0)).x
    # y1 = func(x1, *popt)

    ## now finding the point which is nerest to  x1 and y1
    for i in range(df.shape[0]):
        if df.iloc[i,0] > x1:
            if df.iloc[i,1] > y1:
                x1 = df.iloc[i,0]
                y1 = df.iloc[i,1]
                i = i
    # now finding distance along the gps cordinates
    D =0
    while D < d and i < 400: ## change to length of the df
        x2 = df.iloc[i,0]
        x3 = df.iloc[i+1,0]
        y2 = df.iloc[i,1]
        y3 = df.iloc[i+1,1]
        D += np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        i += 1

    goal = [df.iloc[i,0] , df.iloc[i,1]]
    print(" this is set goal", [30,1])
    return 30 , 30


def is_goal(state, resolution, v, Th , start, df):
    xg , yg = set_goal(v, Th, start , df)
    x , y = state[0], state[1]
    if ((x-xg)**2 + ( y - yg)**2) < resolution**2:
        return True


def check_obstacle(neighbor):
    ####
    # call collision checker in webots
    ####
    return False

# Define the A* algorithm function
def astar(start, resolution): # for now leave obstacle
    open_set = []
    parent = []
    node = [0 , start , parent]
    # cost current state and parernt
    open_set.append(node)

    closed_set = []
    path = []
    b = 0

    while open_set and b < 1000:
        b += 1
        open_set.sort(key=lambda x: x[0])
        current = open_set.pop()
        print(current[0])
        parent = current[2]
        state = current[1]
        ###########
        # how to get the parent -- check
        if is_goal(state, resolution, v, Th , start , df):
            for i in range(len(parent)):
                path.append(parent[i])
            return path

        closed_set.append(current)

        for delta_ in range(0,10,1):
            delta = np.deg2rad(delta_)
            print("this is the delta",delta_)
            T = 1
            neighbor = traj_1(state, delta, T, PLOT=False)
            print("printing neighbour" ,neighbor)

            if check_obstacle(neighbor):
                continue

            if is_goal(neighbor, resolution, v, Th , start , df):
                for i in range(len(parent)):
                    path.append(parent[i])
                return path

            score = current[0] + cost(state, neighbor)  + heuristic(neighbor, v, Th , start, df) # + cost2(neighbor)
            print("printing score",score)

            # change this to within a norm value
            a = True
            for x in closed_set:
                if (x[0] == neighbor).all():
                    a = False
                    if score < x[2]:
                        parent.append(state)
                        node = [score , neighbor , parent]
                        open_set.append(node )
                        #print(" appending neighbour throug a", node)

                    else:
                        continue
            if a:
                parent.append(state)
                node = [score , neighbor , parent]
                open_set.append(node )
                #print(" appending neighbour throug b", node)

    return 'brake'

###########################3
#parameters
###########################

v = 5 #velocity
L = 2.5 #  wheelBase
dt = 0.2 # time step

x0, y0, theta0 = 0, 0, np.deg2rad(0)
delta = 0
Th = 2

resolution = 3

##############################33
df = pd.read_csv('gps_data.csv')
popt = generate_centre_l(df)
# change this data frame to moving average

if __name__=='__main__':
    present = x0, y0 , theta0
    path = astar(present , resolution)

    x = path[0]
    y = path[1]

    plt.scatter(x,y)
    plt.show()




#
# ####################################
# #need to make a trajectory generator given starting and end point
# ####################################
#



# traj_1(x0 , y0, theta0, delta, T)
