#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
import wall_localization
import time 

#setup paths
cwd_path = os.path.dirname(os.path.abspath(__file__)) #gettting the parent directory path

#Setup plot
plt.ion()# seeting up interactive mode

#lists for stroing x position y position and orientation
robot_position_x=[]
robot_position_y=[]
robot_orientation =[]

#defining two axes and figures for plotting 
fig , ax1 = plt.subplots()

#plot robot

pts = ax1.scatter(np.empty((1147,1)),np.empty((1147,1)),c='r')
X_cor = [0.0, 1.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-') # plotting the intial heading from start to another point 
lines = ax1.plot(np.empty((0, 100)), np.empty((0, 100)),color='black', lw=2)
ax1.set(xlim=(-8,8), ylim=(-8,8))
traj = ax1.plot([0, 0], [0, 0])
robotX = -1.5
robotY = 0 
robotTheta = 0
unrobotX = robotX
unrobotY = 0
unrobotTheta = 0 

wallx = []
wally = [] 

trajectory_x = []
trajectory_y = []
def pose_callback(data):
    print("callback")
    #acessing the required global variables 
    global robot_position_x
    global robot_position_y
    global ax1
    global robotX
    global robotY
    global robotTheta
    global unrobotX
    global unrobotY
    global unrobotTheta
    global odometry_data
    global P
    global heading_1 
    global lines
    global pts
    global traj
    global trajectory_x
    global trajectory_y
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,0,0]
    P = np.eye(3)
    lidar_data = data.ranges
    step_size = data.angle_increment
    print('robotx before', robotX)
    robotX, robotY, robotTheta, unrobotX,unrobotY,unrobotTheta,P, plot_vars=  wall_localization.localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P)
    print('robotX after', robotX)
    trajectory_x.append(robotX)
    trajectory_y.append(robotY)
    if True:

        print('robotX, robotY, robotTheta : ', robotX, robotY, robotTheta)        
        robotX+= 10
        robotY+= 5
        time.sleep(0.1)

    if plot_vars !=[]:
  	
        X1,robotX, robotY, robotTheta, walls = plot_vars
        transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
        tick= 0
        for wall in walls:
            p1, p2 = wall
            p1 = list(p1)
            p2 = list(p2)
            p1.append(1)
            p2.append(1)
            p1 = np.matmul(transformation_mat, np.array(p1).reshape((3,1)))
            p2 = np.matmul(transformation_mat, np.array(p2).reshape( (3,1)))

            x = [p1[0][0], p2[0][0]]
            y = [p1[1][0], p2[1][0]]

            lines[tick].set_data(x,y)
            tick +=1
            if tick >= min(len(walls), len(lines)):
                break

# =============================================================================
#         Y1 = [Y[i] for i in range(0, len(Y), 5)]
# =============================================================================
# =============================================================================
#         pts.set_offsets(X1)        
# =============================================================================
        
        heading_x = [robotX, robotX + 1*np.cos(robotTheta)]
        heading_y = [[robotY, robotY + 1*np.sin(robotTheta)]]
        heading_1.set_xdata(heading_x)
        heading_1.set_ydata(heading_y)
        traj.set_data(trajectory_x, trajectory_y)
        print("plotting")
        time.sleep(0.1)

            
            
            
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)

    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/scan", LaserScan, pose_callback,queue_size=1)
    
    plt.show(block=True)
    rospy.spin()
