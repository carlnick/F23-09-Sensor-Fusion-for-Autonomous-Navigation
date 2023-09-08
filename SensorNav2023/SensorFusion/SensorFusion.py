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
import math
# import board
import adafruit_mmc56x3
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import random

START_VALUE:float = -55555555 # placeholder value to start with, out of range of measurements
ZERO:float = 0.0 # zero
MAG_THRESHOLD:float = 50 # Assuming data is in microTeslas
MAG_SUM_THRESHOLD: float = 100
MAG_DELTA_THRESHOLD:float = 1 # Needs to be reviewed once data is obtained

class Vector:
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x
        self.y = y
        self.z = z
    
    def sum(self):
        return (self.x + self.y + self.z)

# Placeholder values, currently unknown what should be within these
threshaccel = Vector()
threshgyro = Vector()
thresh_mag = Vector(x=MAG_THRESHOLD, y=MAG_THRESHOLD, z=MAG_THRESHOLD)

def threshold_mag(data:Vector, last_data:Vector):
    # at beginning, the return is the last stored valid data
    mag_final:Vector = Vector(x=last_data.x, y=last_data.y, z=last_data.z)

    # check sum of axes
    if abs(data.sum()) > MAG_SUM_THRESHOLD:
        return mag_final

    # check each axis for overall and delta thresholds
    if data.x < thresh_mag.x and data.x > -thresh_mag.x:
        if abs(data.x - last_data.x) < MAG_DELTA_THRESHOLD:
            mag_final.x = data.x
        else: print("magnetometer x value exceeds delta threshold")
    else: print("magnetometer x value exceeds threshold")
    if data.y < thresh_mag.y and data.y > -thresh_mag.y:
        if abs(data.y - last_data.y) < MAG_DELTA_THRESHOLD:
            mag_final.y = data.y
        else: print("magnetometer y value exceeds delta threshold")
    else: print("magnetometer y value exceeds threshold")
    if data.z < thresh_mag.z and data.z > -thresh_mag.z:
        if abs(data.z - last_data.z) < MAG_DELTA_THRESHOLD:
            mag_final.z = data.z
        else: print("magnetometer z value exceeds delta threshold")
    else: print("magnetometer z value exceeds threshold")

    return mag_final

def pitch_roll_yaw(accel_data:Vector, mag_data:Vector):
    p_r_y:Vector = Vector()

    # roll and pitch can be calculated using trig with the accelerometer data
    roll = math.atan2(accel_data.y, accel_data.z)
    pitch = math.atan2(-accel_data.x, math.sqrt(accel_data.y*accel_data.y + accel_data.z*accel_data.z))

    # yaw calculation with hard iron error considered
    # yaw_numer: float = ((mag_data.z - hard_iron.z) * math.sin(roll)) - ((mag_data.y-hard_iron.y) * math.cos(roll))
    # yaw_denom:float = ((mag_data.x - hard_iron.x) * math.cos(pitch)) + ((mag_data.y-hard_iron.y) * math.sin(pitch)*math.sin(roll)) + ((mag_data.z - hard_iron.z) * math.sin(pitch) * math.cos(roll))

    # yaw calculation without hard iron error
    yaw_numer: float = (mag_data.z * math.sin(roll)) - (mag_data.y * math.cos(roll))
    yaw_denom:float = (mag_data.x * math.cos(pitch)) + (mag_data.y * math.sin(pitch)*math.sin(roll)) + (mag_data.z * math.sin(pitch) * math.cos(roll))
    yaw = math.atan2(yaw_numer, yaw_denom)
    # add it to the pitch roll yaw vector and return
    p_r_y.x = pitch
    p_r_y.y = roll
    p_r_y.z = yaw

    return p_r_y

def quaternion(p_r_y:Vector):
    quaternion:np.array
    # calculate the quaternion component of each euler angle
    quaternion_pitch:np.array = [math.cos(p_r_y.x/2), math.sin(p_r_y.x/2), 0, 0]
    quaternion_roll:np.array = [math.cos(p_r_y.y/2), 0, math.sin(p_r_y.y/2), 0]
    quaternion_yaw:np.array = [math.cos(p_r_y.z/2), 0, 0, math.sin(p_r_y.z/2)]
    # combine them by multiplying
    quaternion = np.multiply(np.multiply(quaternion_yaw, quaternion_pitch), quaternion_roll);

    return quaternion


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

    # accelerometer = Vector(sensor_imu.acceleration)
    # gyroscope = Vector(sensor_imu.gyro)
    # magnetometer:Vector = Vector(sensor_mag.*something*)
    accelerometer = Vector()
    gyroscope = Vector()
    magnetometer = Vector()
    quat:np.array = [0, 0, 0, 1]

    # Time propagation tracking variables
    last_mag_reading = Vector()
    last_acc_reading = Vector()
    last_gyr_reading = Vector()
    
    #looped code
    while True:
        # current_mag = sensor_mag.magnetic
        magnetometer = threshold_mag(magnetometer, last_mag_reading)

        p_r_y:Vector = Vector()

        p_r_y = pitch_roll_yaw(accel_data=accelerometer, mag_data=magnetometer)
        
        # below is part of complementary filter for filtering new data 
        # and for combining gyroscope data and magnetometer data
        # filteredData = (1-weight)*filteredData + weight*newData
        # fusedData = (1-weight)*gyroData + weight*accelMagData
        
    
        quat = np.multiply(quat, quaternion(p_r_y=p_r_y))

        # print(sensor_imu.acceleration)
        # print(sensor_imu.gyro)

        magnetometer.x = random.random()*5
        magnetometer.y = random.random()*5
        magnetometer.z = random.random()*5

        # last_acc_reading, last_gyr_reading = IMUthreshold(accelerometer, gyroscope, last_acc_reading, last_gyr_reading)
        # get the new acceleration and gyro readings
        time.sleep(1.0)