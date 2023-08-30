##################################################
# This is the main file for the Sensor Fusion Algorithm 
# 
# main at bottom, worker classes above
# 
# Authors: Marlin Spears, Carl Nicklas
# 
###################################################

import numpy as np
import time
# import board
import adafruit_mmc56x3
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import random

START_VALUE:float = -55555555
MAG_THRESHOLD:float = 100 # Assuming data is in microTeslas
MAG_DELTA_THRESHOLD:float = 1 # Needs to be reviewed once data is obtained

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

# Placeholder values, currently unknown what should be within these
threshaccel = Vector(0.0, 0.0, 0.0)
threshgyro = Vector(0.0, 0.0, 0.0)

def threshold_mag(data, last_data):
    if data.x < -100 or data.x > 100:
        return "data bad"

    if last_data != START_VALUE:
        if abs(data - last_data) > 5:
            return "data bad"
        
    return "data good"

# Compare the acceleromter and gyroscope values to their thresholds and sets their values to the previous accelerometer and gyroscope vectors
def IMUthreshold(accelerometer, gyroscope, accel, gyro):
    if accelerometer.x <= threshaccel.x:
        accel.x = accelerometer.x
    if accelerometer.y <= threshaccel.y:
        accel.y = accelerometer.y
    if accelerometer.z <= threshaccel.z:
        accel.z = accelerometer.z

    if gyroscope.x <= threshgyro.x:
        gyro.x = gyroscope.x
    if gyroscope.y <= threshgyro.y:
        gyro.y = gyroscope.y
    if gyroscope.z <= threshgyro.z:
        gyro.z = gyroscope.z

    return accel, gyro
    

if __name__ == "__main__":
    # instances of each board 
    # i2c_imu = board.I2C()
    # sensor_imu = LSM6DSOX(i2c_imu)
    # i2c_mag = board.I2C()
    # sensor_mag = adafruit_mmc56x3.MMC5603(i2c_mag)

    accelerometer = Vector(sensor_imu.acceleration)
    gyroscope = Vector(sensor_imu.gyro)
    magnetometer = Vector(sensor_mag.magnetic)

    # Time propagation tracking variables
    last_mag_reading: float = -55555
    # last_acc_reading: float = -55555
    # last_gyr_reading: float = -55555
    last_acc_reading = Vector(0.0, 0.0, 0.0)
    last_gyr_reading = Vector(0.0, 0.0, 0.0)
    
    current_mag = 0
    #looped code
    while True:
        # current_mag = sensor_mag.magnetic
        print(threshold_mag(current_mag, last_mag_reading))
        # print(sensor_imu.acceleration)
        # print(sensor_imu.gyro)
        last_mag_reading = current_mag
        current_mag += random.random()*5
        last_acc_reading, last_gyr_reading = IMUthreshold(accelerometer, gyroscope, last_acc_reading, last_gyr_reading)
        # get the new acceleration and gyro readings
        time.sleep(1.0)