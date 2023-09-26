
import numpy as np

dt = 0.1
v = 0

x = np.array([[0],  # Initial position
              [0]]) # Initial velocity
P = np.array([[1, 0],  # Initial position covariance
              [0, 1]]) # Initial velocity covariance

A = np.array([[1, dt],  # State transition matrix
              [0, 1]])

B = np.array([[0] ,  # Control matrix
              [dt]])
u = np.array([[0]])  # Control input (change in position)

H_odo = np.array([[1 , 0],[0,1]])  # Measurement matrix for odometry
H_lidar = np.array([[1 , 0],[0,1]]) # Measurement matrix for lidar

R_odometry = np.array([[0.1, 0],
                       [0, 0.2]])  # Odometry measurement noise covariance
R_lidar = np.array([[0.01, 0],
                    [0, 0.5]])    # Lidar measurement noise covariance

Q = np.array([[0.1, 0],
              [0, 0.01]]) # Define process noise covariance

# Ask winston for R and Q matrix values 

z_odometry = np.array([[2],[0]])  # Simulated odometry measurement
z_lidar = np.array([[2.1],[1]])    # Simulated lidar measurement

#Kalman filer loop
for _ in range(50):

    # Prediction
    x_hat = np.dot(A, x) + np.dot(B, u)  
    P_hat = np.dot(np.dot(A, P), A.T) + Q

    # Kalman Gain for odometry
    K_odometry = np.dot(np.dot(P_hat, H_odo.T), np.linalg.inv(np.dot(np.dot(H_odo, P_hat), H_odo.T) + R_odometry))

    # Kalman Gain for lidar
    K_lidar = np.dot(np.dot(P_hat, H_lidar.T), np.linalg.inv(np.dot(np.dot(H_lidar, P_hat), H_lidar.T) + R_lidar))

    # Update using both measurements
    x = x_hat + 0.5*np.dot(K_lidar, (z_lidar - np.dot(H_lidar, x_hat))) +  0.5*np.dot(K_odometry, (z_odometry - np.dot(H_odo, x_hat)))
    P = P_hat - 0.5*np.dot(np.dot(K_lidar , H_lidar), P_hat)  - 0.5*np.dot(np.dot(K_odometry , H_odo), P_hat)

    # Simulate new measurements (replace this with actual sensor readings)
    # remove this section once measurements are available 
    v = v+ u[0][0]*dt
    xd = v*dt + x[0][0]
    z_odometry = np.array([[xd + np.random.normal(0, 0.1)],[v + 0.05 * np.random.normal(0, 1)]])
    z_lidar = np.array([[xd + np.random.normal(0, 0.01)],[v + 0.3*np.random.normal(0, 1)]])

    # till here 
    # get odometry data
    # to z_odo
    
    # get Lidar data
    # to z_lidar
    
    # Print estimated position
    print(f"Estimated Position:x= {x[0][0]}  v = {x[1][0]}")
    # print(f" this is k for lidar {K_lidar}") 
    # print(f"this is k for odometry { K_odometry}")
    # print(f"this is p {P}")

    # Apply control (in this case, a constant change in position)
    u = np.array([[1]])
