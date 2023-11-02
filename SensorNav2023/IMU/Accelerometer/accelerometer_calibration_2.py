import time
import sys
import numpy
import csv
import datetime
import matplotlib.pyplot
from scipy.optimize import curve_fit
from board import I2C
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from typing import Literal

ALLOWED_PORTS = Literal[0, 1, 2, 3, 4, 5, 6, 7]
CALIBRATION_MEASUREMENTS = 1000


def initialize_sensors(imu_ports: list[ALLOWED_PORTS]) -> list[LSM6DSOX]:
    multiplexer = TCA9548A(I2C())
    sensors = []
    for port in imu_ports:
        sensors.append(LSM6DSOX(multiplexer[port]))

    return sensors


def get_raw_data_avg(imu):
    acceleration_sums = numpy.zeros(3)
    for _ in range(CALIBRATION_MEASUREMENTS):
        acceleration_sums[0] += imu.acceleration[0]
        acceleration_sums[1] += imu.acceleration[1]
        acceleration_sums[2] += imu.acceleration[2]

    acceleration_averages = acceleration_sums / float(CALIBRATION_MEASUREMENTS)

    return acceleration_averages


def get_raw_data_row(imus, matrices, row):
    for index, imu in enumerate(imus):
        matrices[index][row] = get_raw_data_avg(imu)


if __name__ == '__main__':
    imus = initialize_sensors([1, 4, 7])

    imu0_raw = numpy.zeros((6, 3))
    imu1_raw = numpy.zeros((6, 3))
    imu2_raw = numpy.zeros((6, 3))

    calibration_directions = ['x-axis up', 'x-axis down', 'y-axis up', 'y-axis down', 'z-axis up', 'z-axis down']

    for row in range(imu0_raw.shape[0]):
        input(f"Orient system with {calibration_directions[row]} and press enter to continue")
        get_raw_data_row(imus, [imu0_raw, imu1_raw, imu2_raw], row)

    print("IMU0:")
    print(imu0_raw)
    print("IMU1:")
    print(imu1_raw)
    print("IMU2:")
    print(imu2_raw)
