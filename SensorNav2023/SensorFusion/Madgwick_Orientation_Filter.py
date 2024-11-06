######## Imports ########
#############################################################
import time
import random
import numpy as np
from time import perf_counter
import matplotlib.pyplot as plt
from SensorFusion.Vector import Vector
from SensorFusion.Quaternion import Quaternion
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer


class Madgwick_Orientation_Filter:
    def __init__(self, q, gyr, acc, mag=None, frequency=100.0, MARG=False):
        # Initalize Gain
        if MARG: # If MARG use gain of 0.041
            self.gain = 0.041
        else:
            # If only IMU use gain of 0.033
            self.gain = 5.0

        # Initialize Sampling Rate
        self.freq = frequency

        # Initialize Ouput Quaternion
        self.Q = q
        

    def update_IMU(self, gyro, accel):
        # Initialize Variable from Inputs
        w, i, j, k = self.Q
        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel

        # Normalize Acceleration
        norm_acc = np.linalg.norm([ax, ay, az])
        ax, ay, az = ax/norm_acc, ay/norm_acc, az/norm_acc

        # Gradient Descent
        F_g = np.array([
            2.0*(i*k - w*j) - ax,
            2.0*(w*i + j*k) - ay, 
            2.0*(0.5 - i**2 - j**2) - az])

        # Jacobian
        J_g = np.array([
            [-2.0*j, 2*k, -2.0*w, 2.0*i],
            [2.0*i, 2.0*w, 2.0*k, 2.0*j],
            [0.0, -4*i, -4*j, 0]])

        Q_dot = np.array([
            -i*gx - k*gy - k*gz,
             w*gx + j*gz - k*gy,
             w*gy - i*gz + k*gx,
             w*gz + i*gy - j*gx])
        Q_dot *= 0.5

        gradient_f = J_g.T @ F_g

        self.Q += (Q_dot - self.gain * gradient_f) * (1.0 / self.freq)
        self.Q /= np.linalg.norm(self.Q)

        return self.Q


    def update_MARG(self, gyro, accel, mag):
        w, i, j, k = self.Q
        gx, gy, gz = np.radians(gyro)
        ax, ay, az = accel
        mx, my, mz = mag

        # Normalize Acceleration
        norm_acc = np.linalg.norm([ax, ay, az])
        ax, ay, az = ax/norm_acc, ay/norm_acc, az/norm_acc

        # Normalize Magnetometer Readings
        norm_mag = np.linalg.norm([mx, my, mz])
        mx, my, mz = mx/norm_mag, my/norm_mag, mz/norm_mag

        # Gradient Descent
        F_g = np.array([
            2.0*(i*k - w*j) - ax,
            2.0*(w*i + j*k) - ay, 
            2.0*(0.5 - i**2 - j**2) - az])

        # Jacobian
        J_g = np.array([
            [-2.0*j, 2*k, -2.0*w, 2.0*i],
            [2.0*i, 2.0*w, 2.0*k, 2.0*j],
            [0.0, -4*i, -4*j, 0]])

        # Gradient Descent
        F_b = np.array([
            2.0*mx*(0.5 - j**2 - k**2) + 2.0*mz*(i*k - w*j) - mx,
            2.0*mx*(i*j - w*k) + 2.0*mz*(w*i + j*k) - my,
            2.0*mx*(w*j + i*k) + 2.0*mz*(0.5 - i**2 - j**2) - mz])

        # Jacobian
        J_b = np.array([
            [-2.0*mz*j, 2.0*mz*k, -4.0*mx*j - 2.0*mz*w, -4.0*mx*k + 2.0*mz*i],
            [-2.0*mx*k + 2.0*mz*i, 2.0*mx*j + 2.0*mz*w, 2.0*mx*i + 2.0*mz*k, -2.0*mx*w + 2.0*mz*j],
            [2.0*mx*j, 2.0*mx*k - 4.0*mz*i, 2.0*mx*w - 4.0*mz*j, 2.0*mx*i]])

        F_g_b = np.concatenate((F_g, F_b))
        J_g_b = np.concatenate((J_g, J_b), axis=0)

        gradient = J_g_b.T @ F_g_b

        Q_dot = 0.5 * np.array([
            -i*gx - k*gy - k*gz,
             w*gx + j*gz - k*gy,
             w*gy - i*gz + k*gx,
             w*gz + i*gy - j*gx])

        self.Q += (Q_dot - self.gain * gradient) * (1.0 / self.freq)
        self.Q /= np.linalg.norm(self.Q)

        return self.Q
        
