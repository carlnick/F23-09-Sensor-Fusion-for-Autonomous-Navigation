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

def threshold_mag(data, last_data):
    if data.x < -100 or data.x > 100:
        return "data bad"

    if last_data != START_VALUE:
        if abs(data - last_data) > 5:
            return "data bad"
        
    return "data good"
    

if __name__ == "__main__":
    # instances of each board 
    # i2c_imu = board.I2C()
    # sensor_imu = LSM6DSOX(i2c_imu)
    # i2c_mag = board.I2C()
    # sensor_mag = adafruit_mmc56x3.MMC5603(i2c_mag)

    # Time propagation tracking variables
    last_mag_reading: float = START_VALUE
    last_acc_reading: float = START_VALUE
    last_gyr_reading: float = START_VALUE
    
    current_mag = 0
    #looped code
    while True:
        # current_mag = sensor_mag.magnetic
        print(threshold_mag(current_mag, last_mag_reading))
        # print(sensor_imu.acceleration)
        # print(sensor_imu.gyro)
        last_mag_reading = current_mag
        current_mag += random.random()*5
        time.sleep(1.0)