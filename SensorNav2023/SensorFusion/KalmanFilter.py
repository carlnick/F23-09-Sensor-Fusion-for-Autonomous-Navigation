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

************************************************************************************* '''

import numpy as np
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

class QuaternionKalmanFilter:
    def __init__(self, initial_state):
        # State estimate, quaternion
        self.state = initial_state
        
        # Initial covariance matrix
        self.P = np.eye(4) * 0.1
        
        # Process noise covariance
        self.Q = np.eye(4) * 0.01
        
        # Measurement noise covariance
        self.R = np.eye(9) * 0.1
        
    def predict(self, omega, dt):
        omega_quat = Quaternion.from_angular_velocity(omega, dt)
        dq = omega_quat.multiply(self.state)

        self.state = (self.state + dq).normalize()

        G = np.array([
            [0, -omega.x, -omega.y, -omega.z],
            [omega.x, 0, -omega.z, omega.y],
            [omega.y, omega.z, 0, omega.x],
            [omega.z, -omega.y, omega.x, 0]])
        
        # Update covariance
        F = np.eye(4) + 0.5 * G * dt
        self.P = F @ self.P @ F.T + self.Q
        #self.P = (self.P + self.P.T) / 2
        max_P = 1e25
        self.P = np.clip(self.P, -max_P, max_P)
    
    def correct(self, measurement):
        state_as_array = self.state.to_array()
        #print(state_as_array.shape)

        # H matrix maps the state estimate to the measurement space
        H = np.zeros((9, 4))
        H[:4, :4] = np.eye(4)
        
        # Measurement residual
        y = measurement - H @ state_as_array
        #print(measurement.shape)
        
        # Residual covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        #K = self.P @ H.T @ np.linalg.pinv(S)
        #epsilon = 1e-5
        #S += epsilon * np.eye(S.shape[0])

        #K = self.P @ H.T @ np.linalg.pinv(S)

        try:
            U, sigma, VT = np.linalg.svd(S)
            sigma_inv = np.array([1/s if s > 1e-5 else 0 for s in sigma])
            S_inv = VT.T @ np.diag(sigma_inv) @ U.T
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            print("ehfl")
            epsilon = 1e-2
            S += epsilon * np.eye(S.shape[0])
            S_inv = np.linalg.inv(S)
            K = self.P @ H.T @ S_inv

        
        new_state_array = state_as_array + K @ y
        self.state = Quaternion(*new_state_array)

        # Update covariance
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_state(self):
        return self.state

