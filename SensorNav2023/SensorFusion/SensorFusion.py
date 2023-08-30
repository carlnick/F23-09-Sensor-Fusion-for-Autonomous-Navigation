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

def threshold_mag(data, last_data):
    # Assuming  data is in microTeslas
    if data < -100 or data > 100:
        return "data bad"

    if last_data != -55555:
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
    last_mag_reading: float = -55555
    last_acc_reading: float = -55555
    last_gyr_reading: float = -55555
    
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