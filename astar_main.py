import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.optimize import curve_fit , minimize_scalar
from scipy.spatial.distance import euclidean
import math
import time
from Astar import astar , bicycle_model

class car:
    resolution = np.sqrt(0.4)

    T = 10 # time horizon for planning in front of car
    tme = 0.001 #  pause time for plotting
    n = 4  # number for moving average
    v = 6 #velocity
    WB = 2.5 #  wheelBase
    Th = 1   # time step fro one trajectory generation
    # dt = Th/3 # time step

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

def stanley_control(x, y, yaw, v, target):
    # Calculate the cross-track error
    current_position = np.array([x, y])
    target_position = np.array([target[0], target[1]])
    diff = target_position - current_position
    e = np.sqrt(diff[0]**2 + diff[1]**2)

    # Calculate the heading error
    angle_error = yaw - np.arctan2(diff[1], diff[0])

    # Correct for angle wrapping
    if angle_error > np.pi:
        angle_error -= 2 * np.pi
    elif angle_error < -np.pi:
        angle_error += 2 * np.pi

    # Calculate the control input
    delta = np.arctan2(0.5* e, v) + angle_error - np.arctan2(0.5 * L, v)

    return delta

def traj(state, Th, PLOT= False):
    # print("now traj_1")
    n = 2
    dt = Th/n
    trajectory = [state[0:4]]
    print("this is the initial point of trajectory")
    delta = 0
    ds = 0
    state_p = trajectory[-1]
    x1,y1 = state_p[0:2]
    x,y,theta,delta = bicycle_model(state_p,delta, v, L, dt)
    ds += np.sqrt((x-x1)**2 + (y - y1)**2)
    trajectory.append([np.round(x,3),np.round(y,3),theta,delta])

    trajectory = np.array(trajectory)
    if PLOT == True:
        plt.plot(trajectory[:,0], trajectory[:,1])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.title('Trajectory of Kinematic Bicycle Model')
        plt.show()
        plt.pause(0.1)
    print("this is the trajectory", trajectory)
    print("this is the output from the traj code", trajectory[-1])
    return trajectory[-1] , ds


if __name__=='__main__':
    T = car.T # time horizon for planning in front of car
    tme = car.tme #  pause time for plotting
    n = car.n  # number for moving average
    v = car.v #velocity
    L = car.WB #  wheelBase
    Th = car.Th   # time step fro one trajectory generation
    resolution = np.sqrt(0.4)

    coord = track(n)
    x0, y0, theta0 , delta, distance =  0, 0, np.deg2rad(90), np.deg2rad(0), 0
    present = x0, y0 , theta0, delta, distance

    obstacle = np.array([[8.73,45.3], [13.25, 51.06], [4.89,43.87],[72.73,59.81],[139.84,43.79],[148.86,-12.81],[150.8,-63.15], [147.08,13.61],[148.46,-40.36],[105.17,-88.96],[64.23,-88.96]])
    for points in obstacle:
        x , y = points
        plt.scatter(x,y, color='black', marker = '*')
    b = 0

    while distance < 420 and b <5:

        start_time = time.time()
        path = astar(present , resolution, coord , Th)
        end_time = time.time()
        # print(end_time - start_time , "this is the time taken for each iteration")
        if path != 'brake':
            xp = [row[0] for row in path]   # extract 0th column
            yp = [row[1] for row in path]
            plt.plot(xp[-2:-1],yp[-2:-1])
            theta_res = [row[2] for row in path]
            del_res = [row[3] for row in path]
            delta_steering = stanley_control(xp[-1], yp[-1], theta_res[-1], v , [xp[-2], yp[-2]])
            # print("this is the 2nd state in list",xp[-2], yp[-2], delta[-1] )
            # plt.scatter(xp[-2], yp[-2])
            # print(delta_steering, " this is the steering angle output of stanley")
        else:
            b+= 1
            print('brake')
            continue
        state = [xp[-1], yp[-1], theta_res[-1], del_res[-1] ]
        trajectory , ds = traj(state, Th, PLOT= True)
        # # print(trajectory, " this is the last point of trajectory generated")
        distance += ds
        if distance >420:
            print("!!!!!!!!!!!!!!!!########################!!!!!!!!!!!!!!!!!!!!!!!")
        # print("this is the ds after stanley", ds )
        present = trajectory[0], trajectory[1] , trajectory[2], del_res[-2], distance
        # present = xp[-2] , yp[-2] , theta_res[-2], del_res[-2] , distance
        print("this is the next state ", present)

    plt.show()
    plt.pause(30)
