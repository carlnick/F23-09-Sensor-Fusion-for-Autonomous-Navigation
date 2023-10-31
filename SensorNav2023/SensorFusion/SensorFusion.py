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

#import class types
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

START_VALUE:float = -55555555 # placeholder value to start with, out of range of measurements
ZERO:float = 0.0 # zero
MAG_THRESHOLD:float = 50 # Assuming data is in microTeslas
MAG_SUM_THRESHOLD: float = 100
MAG_DELTA_THRESHOLD:float = 1 # Needs to be reviewed once data is obtained


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

def quat_mult(p:np.array, q:np.array):
    quaternion:np.array = [0,0,0,0]

    quaternion[0] = (p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3])
    quaternion[1] = (p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2])
    quaternion[2] = (p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1])
    quaternion[3] = (p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0])

    return quaternion

def orientation(accel_data:Vector, mag_data:Vector = None):
    qAccelerometer = Quaternion()
    qMagnetometer = Quaternion()
    if(mag_data is None):
        qResult = Quaternion()
        p_r_y = accel_data
        # calculate the quaternion components based on euler angle trig
        # note: function assumes angles in radians
        qResult.set_q0(math.cos(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.cos(p_r_y.z/2)
                       + math.sin(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q1(math.sin(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.cos(p_r_y.z/2)
                        - math.cos(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q2(math.cos(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.cos(p_r_y.z/2)
                        + math.sin(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q3(math.cos(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.sin(p_r_y.z/2)
                       - math.sin(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.cos(p_r_y.z)/2)

        return qResult
    # calculate the quaternion components based on euler angle trig
    # note: function assumes angles in radians
    if accel_data.z >= 0:
        qAccelerometer.set_q0(math.sqrt((accel_data.z +1)/2))
        qAccelerometer.set_q1(-1*(accel_data.y)/math.sqrt(2*(accel_data.z +1)))
        qAccelerometer.set_q2((accel_data.x)/math.sqrt(2*(accel_data.z +1)))
        qAccelerometer.set_q3(0)
    else:
        qAccelerometer.set_q0(-1*(accel_data.y)/math.sqrt(2*(1-accel_data.z)))
        qAccelerometer.set_q1(math.sqrt((1-accel_data.z)/2))
        qAccelerometer.set_q2(0)
        qAccelerometer.set_q3((accel_data.x)/math.sqrt(2*(1-accel_data.z)))

    rotatedMagField = Quaternion.rotateMultipy(qAccelerometer.inverse(), mag_data)
    gamma = rotatedMagField.x**2 + rotatedMagField.y**2

    if rotatedMagField.x >= 0:
        qMagnetometer.set_q0(math.sqrt((gamma + rotatedMagField.x * math.sqrt(gamma))/(2*gamma)))
        qMagnetometer.set_q1(0)
        qMagnetometer.set_q2(0)
        qMagnetometer.set_q3(rotatedMagField.y/(math.sqrt(2*(gamma + rotatedMagField.x * math.sqrt(gamma)))))
    else:
        qMagnetometer.set_q0(rotatedMagField.y/(math.sqrt(2*(gamma - rotatedMagField.x * math.sqrt(gamma)))))
        qMagnetometer.set_q1(0)
        qMagnetometer.set_q2(0)
        qMagnetometer.set_q3(math.sqrt((gamma - rotatedMagField.x * math.sqrt(gamma))/(2*gamma)))

    return qAccelerometer, qMagnetometer


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
    
# Find the current position & velocity of the object
def currentPositionVelocity(acceleration, quaternion, prevVel, prevPos, timeDelta):
    gravity = Vector()
    gravity.x = 2 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2)
    gravity.y = 2 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3)
    gravity.z = quaternion.q0**2 - quaternion.q1**2 - quaternion.q2**2 + quaternion.q3**2

    linearAccel = Vector()

    linearAccel.x = acceleration.x - gravity.x
    linearAccel.y = acceleration.y - gravity.y
    linearAccel.z = acceleration.z - gravity.z

    velocity = Vector()

    velocity.x = prevVel.x + linearAccel.x * timeDelta
    velocity.y = prevVel.y + linearAccel.y * timeDelta
    velocity.z = prevVel.z + linearAccel.z * timeDelta

    position = Vector()

    position.x = prevPos.x + prevVel.x * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.x
    position.y = prevPos.y + prevVel.y * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.y
    position.z = prevPos.z + prevVel.z * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.z

    return position, velocity



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
    quat:np.array = [1.0, 0.0, 0.0, 0.0]

    # Time propagation tracking variables
    last_mag_reading = Vector()
    last_acc_reading = Vector()
    last_gyr_reading = Vector()
    
    # TEST CODE FOR QUATERNION FUNCTION
    # for i in range(0, 50):
    #     test_pry1 = Vector(i,i,i)
    #     test_pry2 = Vector(i,0.0,0.0)
    #     test_pry3 = Vector(0.0,i,0.0)
    #     test_pry4 = Vector(0.0,0.0,i)
    #     print(orientation(test_pry1))
    #     print(orientation(test_pry2))
    #     print(orientation(test_pry3))
    #     print(orientation(test_pry4))
     # TEST CODE FOR QUATERNION FUNCTION


    #looped code
    while True:
        # current_mag = sensor_mag.magnetic
        magnetometer = threshold_mag(magnetometer, last_mag_reading)

        p_r_y:Vector = Vector()

        p_r_y = pitch_roll_yaw(accel_data=accelerometer, mag_data=magnetometer)

        # Complementary filter:
        
        
        # below is part of complementary filter for filtering new data 
        # and for combining gyroscope data and magnetometer data
        # error magnitude = ((magnitude of local frame acceleration - 9.81 (G)) / 9.81 (G))
        # if (em > 0.2) weight = 0
        # else if (em > 0.1) weight = (-1 * em * 10 + 2) * em **SHOULD PROBABLY BE CHANGED FOR OUR SYSTEM
        # else weight = 1 * em                                **BOTH SLOPE AND CUTOFFS
        #                                                     **similar system could maybe be used for gyroscope
        #                                                     **what is global reference for that? equivalent of gravity = 9.81
        # filteredData = (1-weight)*filteredData + weight*newData
        # fusedData = (1-weight)*gyroData + weight*accelMagData
        # new data is result of quaternion call
        # filtered data starts with original [1,0,0,0] and is edited each time around
        # weight is detailed above
        # gyro data is gyro quat and accelmag data is result of accelmag quat
        # second one is most likely what we will apply
    
        quat = np.multiply(quat, orientation(p_r_y=p_r_y))

        # print(sensor_imu.acceleration)
        # print(sensor_imu.gyro)

        magnetometer.x = random.random()*5
        magnetometer.y = random.random()*5
        magnetometer.z = random.random()*5

        # last_acc_reading, last_gyr_reading = IMUthreshold(accelerometer, gyroscope, last_acc_reading, last_gyr_reading)
        # get the new acceleration and gyro readings
        time.sleep(1.0)
