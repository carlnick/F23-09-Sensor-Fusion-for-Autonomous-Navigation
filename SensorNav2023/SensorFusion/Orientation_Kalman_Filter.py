import numpy as np
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

class Orientation_Kalman_Filter:
    def __init__(self, initial_quaternion, gyro_var, mag_var, current_time):

        # Initialize Variables for the initial Quaternion
        self.w = initial_quaternion[0] 
        self.i = initial_quaternion[1] 
        self.j = initial_quaternion[2] 
        self.k = initial_quaternion[3] 

        # State Estimate
        self.X = np.array([self.w, self.i, self.j, self.k])

        # Initial State Error Covariace Matrix
        self.P = np.identity(4)

        # Initial Observation Matrix
        self.H = np.identity(4)

        # Process noise covariance matrix - Gyroscope
        self.Q = np.array([
            [gyro_var, 0, 0, 0], 
            [0, gyro_var, 0, 0],
            [0, 0, mag_var, 0],
            [0, 0, 0, mag_var]])

        # Measurement noise covariance matrix - Magnetometer
        self.R = np.array([
            [gyro_var, 0, 0, 0],
            [0, gyro_var, 0, 0], 
            [0, 0, mag_var, 0],
            [0, 0, 0, mag_var]])

        # Initial Kalman Gain
        self.K = None

        # Initial Measurment Vector
        self.z = None

        # Initial Dynamics Model
        self.A = None

        # Initial Input Matrix
        self.B = None

        # Identity Matrix
        self.I = np.identity(4)

        # Current Time
        self.current_time = current_time
        
        # Previous Time
        self.previous_time = None

        # Delta T
        self.delta_t = None


    def predict(self, gyro_reading, timestamp):
        # If there is no previous time set delta_t to 1e-5
        if self.previous_time is None:
            self.delta_t = 1e-5
        else:
            # Delta_t is the change in time
            self.delta_t = timestamp - self.previous_time

        # Extract gyroscope readings
        gx = gyro_reading[0]
        gy = gyro_reading[1]
        gz = gyro_reading[2]

        # Dynamics Model
        self.A = np.array([
            [0, -gx, -gy, -gz],
            [gx, 0, gz, -gy],
            [gy, -gz, 0, gx],
            [gz, gy, -gx, 0]])
        self.A = self.A * (self.delta_t * 0.5)
        self.A = self.I + self.A
        
        # Calculate prediction for State - X
        self.X = np.matmul(self.A, self.X)

        # Calculate prediction for State Error Covariance - P
        tmp = np.matmul(self.A, self.P)
        A_T = np.transpose(self.A)
        self.P = np.add(np.matmul(tmp, A_T), self.Q)

        # Update timestamps
        self.previous_time = self.current_time
        self.current_time = timestamp

        return self.X

    def update(self, madgwick_reading):
        # Initialize quaternion variables
        w = madgwick_reading[0] 
        i = madgwick_reading[1] 
        j = madgwick_reading[2] 
        k = madgwick_reading[3] 

        # Measurment Vector
        self.z = np.array([w, i, j, k])

        # Update Observation Matrix
        self.H = np.eye(4)

        # calculate Kalman Gain - K
        tmp_K = self.H @ self.P @ self.H.T + self.R
        self.K = self.P @ self.H.T @ np.linalg.inv(tmp_K)

        # Calculate Updated State
        tmp_X = np.subtract(self.z, np.matmul(self.H, self.X))
        self.X = np.add(self.X, np.matmul(self.K, tmp_X))

        # Calculate Updated State Error Covariance
        tmp_P = np.subtract(self.I, np.matmul(self.K, self.H))
        self.P = np.matmul(tmp_P, self.P)

        return self.X

    def update_var(self, gyro_var, mag_var):
        self.Q = np.array([
            [gyro_var, 0, 0, 0], 
            [0, gyro_var, 0, 0],
            [0, 0, mag_var, 0],
            [0, 0, 0, mag_var]])

        self.R = np.array([
            [gyro_var, 0, 0, 0],
            [0, gyro_var, 0, 0], 
            [0, 0, mag_var, 0],
            [0, 0, 0, mag_var]])

