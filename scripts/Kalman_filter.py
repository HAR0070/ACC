
import numpy as np

dt = 0.1

x = np.array([[0],  # Initial position
              [0]]) # Initial velocity
P = np.array([[1, 0],  # Initial position covariance
              [0, 1]]) # Initial velocity covariance

A = np.array([[1, dt],  # State transition matrix
              [0, 1]])

B = np.array([[0],  # Control matrix
              [dt]])
u = np.array([[0]])  # Control input (change in position)

H_odo = np.array([[1 , 1]])  # Measurement matrix for odometry
H_lidar = np.array([[1 , 0]]) # Measurement matrix for lidar

R_odometry = np.array([[0.1]])  # Odometry measurement noise covariance
R_lidar = np.array([[0.01]])    # Lidar measurement noise covariance

Q = np.array([[0.01, 0],
              [0, 0.01]]) # Define process noise covariance

z_odometry = np.array([[2]])  # Simulated odometry measurement
z_lidar = np.array([[3]])    # Simulated lidar measurement

# get odometry data
"""
Get the odometry data
"""
# get Lidar data

# develop motion model

# write a kalman filter code

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
    x = x_hat + np.dot(K_odometry, (z_odometry - np.dot(H_odo, x_hat))) + np.dot(K_lidar, (z_lidar - np.dot(H_lidar, x_hat)))
    P = P_hat - np.dot(np.dot(H_odo.T, K_odometry), P_hat) - np.dot(np.dot(H_lidar.T, K_lidar), P_hat)

    # Simulate new measurements (replace this with actual sensor readings)
    z_odometry = np.array([[2 + np.random.normal(0, 0.1)]])
    z_lidar = np.array([[3 + np.random.normal(0, 0.01)]])

    # Print estimated position
    print(f"Estimated Position: {x[0][0]}")

    # Apply control (in this case, a constant change in position)
    u = np.array([[1]])
