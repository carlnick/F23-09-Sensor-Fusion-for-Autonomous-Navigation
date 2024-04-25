from time import time, perf_counter
from SensorFusion.Vector import Vector
from SensorFusion.Quaternion import Quaternion
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
from SensorFusion.KalmanFilter import QuaternionKalmanFilter
import numpy as np

# Initialize imu and magnetometer
imu = IMU([1, 4, 7])
mag = Magnetometer()

# Initial quaternion state from sensor readings (example)
vAccelerometer = Vector(*imu.get_acceleration()).normalize()
vMagnetometer = Vector(*mag.get_magnetic()).normalize()
qInitial = Quaternion.from_two_vectors(vAccelerometer, vMagnetometer)

# Initialize Kalman filter with initial quaternion
kf = QuaternionKalmanFilter(qInitial)

# Main loop
num_samples = 0
total_time = 0.0
while True:
    start_time = perf_counter()

    # Sensor updates
    vGyroscope = Vector(*imu.get_gyroscope())
    vAccelerometer = Vector(*imu.get_acceleration()).normalize()
    vMagnetometer = Vector(*mag.get_magnetic()).normalize()

    # Prediction step
    kf.predict(vGyroscope, time() - start_time)

    # Measurement for correction
    # Combining normalized accelerometer and magnetometer readings
    measurement = np.concatenate([vGyroscope.to_array(), vAccelerometer.to_array(), vMagnetometer.to_array()])

    # Correction step
    kf.correct(measurement)

    # Current quaternion state
    current_orientation = kf.get_state()
    
    # Display or log the orientation here
    print(current_orientation)

    # Handling time and loop breaks for demonstration
    num_samples += 1
    total_time += (perf_counter() - start_time)
    #if num_samples > 600:  # Or some other condition to exit
        #break

