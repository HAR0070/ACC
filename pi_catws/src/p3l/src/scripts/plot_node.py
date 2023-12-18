#!/usr/bin/env python3
#importing the necessary Libraries required 
import matplotlib.pyplot as plt #for plotting 
import numpy as np # the numerical python library
import rospy #python library for ros
import os # python library for system file path
from geometry_msgs.msg import Twist #importing the messgae for publishing 
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
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
fig, ax1 = plt.subplots()

#plot robot


X_cor = [0.0, 1.0]
Y_cor = [0.0, 0.0]
heading_1, = ax1.plot(X_cor, Y_cor,'b-') # plotting the intial heading from start to another point 
cov_ellipse, = ax1.plot(X_cor, Y_cor, 'b-')
robot_pos_plt = ax1.scatter(0,0,c='g')

lines = ax1.plot(np.empty((0, 100)), np.empty((0, 100)),color='black', lw=2)
ax1.set(xlim=(-8,8), ylim=(-8,8))
traj, = ax1.plot([0, 0], [0, 0])
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
def Str2plot_data(plot_str):
    #print(plot_str.data)
    lines = plot_str.data.split('\n')

    P = np.zeros(shape=(3,3))
    line_list = []
    for line_id, line in enumerate(lines):
        if line_id == 0:
            robotX, robotY, robotTheta = list(map(float, line.split()))
        if line_id == 1:
            P_data = list(map(float, line.split()))
            for i in range(3):
                for j in range(3):
                    P[i, j] = P_data[i*3+ j]
        
        if line_id > 1:
            if line.split():
                p1, p2 = [0,0], [0,0]
                p1[0], p1[1], p2[0], p2[1] = list(map(float, line.split()))
                line_list.append([p1, p2])
                
                
    return robotX, robotY, robotTheta, P, line_list

def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = np.linalg.eig(covariance[0:2,0:2])
        angle = np.arctan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, np.sqrt(eigenvals[0]), np.sqrt(eigenvals[1]))

def ellipse_data(angle, a, b):
    
    thetas = np.linspace(0, 2*np.pi, 40)
    x = a*np.cos(thetas)
    y = b*np.sin(thetas)    
    
    coord = np.array([[x],[y]]).reshape(2, 40)
    rot= np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])    
    
    coord = np.matmul(rot, coord)

    x = coord[0,:]
    y = coord[1,:]
    return x, y
    

    
def pose_callback(data):
    print("callback")
    #acessing the required global variables 
    
    """
    variables to unpack - robot_X, robotY, robotTheta, P, walls
    """
    
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
    global cov_ellipse
    global robot_pos_plt
    #finding th erequired points to be plotted 
    X = []
    Y = []
    odometry_data =[0,0,0]
    
    robotX, robotY, robotTheta, P, walls = Str2plot_data(data)
    
    trajectory_x.append(robotX)
    trajectory_y.append(robotY)


    transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
    tick= 0
    time.sleep(0.1)
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
    angle, a, b = get_error_ellipse(P)
    x, y = ellipse_data(angle, a, b)
    
    heading_x = [robotX, robotX + 1*np.cos(robotTheta)]
    heading_y = [robotY, robotY + 1*np.sin(robotTheta)]
    heading_1.set_xdata(heading_x)
    heading_1.set_ydata(heading_y)
    robot_pos_plt.set_offsets([robotX, robotY])
    
    cov_ellipse.set_xdata(x)
    cov_ellipse.set_ydata(y)
    traj.set_data(trajectory_x, trajectory_y)
    
    print('robotX, robotY, robotTheta :', robotX, robotY, robotTheta)
    time.sleep(0.1)

            
                
if __name__ == '__main__':

    #intialising a node for the vizualisation part 
    rospy.init_node('rover_visualisation', anonymous=True)

    #subscribing the required topic and updating its callback function 
    rospy.Subscriber("/plot_data", String, pose_callback,queue_size=1)
    plt.show(block=True)
    
    rospy.spin()
