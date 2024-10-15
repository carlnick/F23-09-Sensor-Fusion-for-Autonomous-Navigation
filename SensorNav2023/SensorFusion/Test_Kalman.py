
import numpy as np
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

class Test_Kalman_Filter:
    def __init__(self, initial_position, initial_velocity, gps_var, acc_var, current_time):
        # State Estimate
        self.X = np.array([[np.float64(initial_position)], [np.float64(initial_velocity)]])

        # Initial State Error Covariace Matrix
        self.P = np.identity(2)

        # Initial Measurement Matrix
        self.H = np.identity(2)

        # Process noise covariance matrix - Accelerometer
        self.Q = np.array([[acc_var, 0], [0, acc_var]])

        # Measurement noise covariance matrix - GPS
        self.R = np.array([[gps_var, 0], [0, gps_var]])

        # Initial Kalman Gain
        self.K = None

        # Initial Measurment Vector
        self.z = None

        # Initial Dynamics Model
        self.A = None

        # Initial Input Matrix
        self.B = None

        # Initial Input Vector
        self.u = None

        # Identity Matrix
        self.I = np.identity(2)

        # Current Time
        self.current_time = current_time
        
        # Previous Time
        self.previous_time = None

        # Delta T
        self.delta_t = None


    def predict(self, acc_reading, timestamp):

        if self.previous_time is None:
            self.delta_t = 1e-5
        else:
            self.delta_t = timestamp - self.previous_time

        # Dynamics Model
        self.A = np.array([[1.0, self.delta_t], [0.0, 1.0]])

        # Input Matrix
        self.B = np.array([[0.5*self.delta_t**2], [self.delta_t]])

        # Input Vector
        self.u = np.array([[acc_reading]])

        # Calculate prediction for State
        self.X = np.add(np.matmul(self.A, self.X), np.matmul(self.B, self.u))

        # Calculate prediction for State Error Covariance
        tmp = np.matmul(self.A, self.P)
        A_T = np.transpose(self.A)
        self.P = np.add(np.matmul(tmp, A_T), self.Q)

        # Update timestamps
        self.previous_time = self.current_time
        self.current_time = timestamp

    def update(self, gps_pos, velocity):
        # Measurment Vector
        self.z = np.array([[gps_pos], [velocity]])

        # Calculate Kalman Gain
        H_T = np.transpose(self.H)
        tmp_K = np.add(np.matmul(self.H, np.matmul(self.P, H_T)), self.R)
        tmp_K_inv = np.linalg.inv(tmp_K)

        self.K = np.matmul(self.P, np.matmul(H_T, tmp_K_inv))

        # Calculate Updated State
        tmp_X = np.subtract(self.z, np.matmul(self.H, self.X))
        self.X = np.add(self.X, np.matmul(self.K, tmp_X))

        # Calculate Updated State Error Covariance
        tmp_P = np.subtract(self.I, np.matmul(self.K, self.H))
        self.P = np.matmul(tmp_P, self.P)

        print("UPDATE", gps_pos)

    def get_position(self):
        return self.X[0, 0]

    def set_R_gps():
        return None

    def set_R_acc():
        return None

    def set_Q_gps():
        return None

    def set_Q_acc():
        return None

        





