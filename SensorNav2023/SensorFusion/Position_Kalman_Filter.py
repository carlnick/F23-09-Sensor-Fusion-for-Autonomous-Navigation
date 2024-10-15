''' *************************************************************************************
    Variable Definitions
        * K - Kalman Gain (Detirmines how much the predicted state should be adjusted based on the incoming sensor measurments. Weighs the predicted state against the measurment to detirmine the updated estimate) (4x4 matrix)
        * F - State transition matrix (Describes state vectors changes from one step to the next. Models relationships between the different state variable over time) (4x4 matrix)
        * P - State covariance matrix (Uncertainty in the state matrix. Each element represents uncertainty on the variable and uncertainties in correlation to other variables) (4x4 matrix)
        * Q - Process noise covariance matrix (Models the noise in the process of the filter itself which comes from modeling errors and approximations) (4x4 matrix) 
        * R - Measurement noise covariance matrix (Uncertainty of sensor measurements) (9x9 matrix)
        * H - Measurement matrix (Relationship between the state vector and measurment vector. Maps the state space and indicates which aspects can be observed.) (9x4 matrix (or maybe 4x9?))
        * y - Residual (Difference between acutal measurment (from sensor) and predicted measurment)
        * S - Residual Covariance (Expected uncertainty of y) (4x4 matrix)
        * G - Organization of Gyroscope values  /*may need more info update later*/
        * State - State of the system

************************************************************************************* '''

import numpy as np
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

class Position_Kalman_Filter:
    def __init__(self, initial_position, acc_variance, pos_variance):
        # State estimate
        self.X = 0
        
        # Initial state error covariance matrix
        self.P = 1.0

        # Initial predicted state error covariance matrix
        self.P_minus = 1.0

        self.K = np.array([[1], [1]])
        
        # Process noise covariance matrix - Accelerometer
        self.Q = acc_variance
        
        # Measurement noise covariance matrix - GPS
        self.R = pos_variance

        # Identity Matrix
        self.I = 1

        # Initial measurement matrix
        self.H = np.array([0, 1])

    def estimate_state(self, measurement_vector, GPS_used):
        if (GPS_used):
            self.H = np.array([1, 0])
        else:
            self.H = np.array([0, 1])

        # Predict the state error covariance
        self.P_minus = self.P + self.Q

        # Calculate the Kalman Gain
        self.K = np.array([[self.P_minus], [self.P_minus]]) + self.H.T @ np.linalg.inv(self.H * self.P_minus * self.H.T + self.R)

        # Update the State Estimate
        self.X = self.X + self.K @ (measurement_vector - self.H * self.X)

        # Update the State Error Covariance 
        self.P = (self.I - self.K @ self.H) @ (self.P_minus)

        return self.X


