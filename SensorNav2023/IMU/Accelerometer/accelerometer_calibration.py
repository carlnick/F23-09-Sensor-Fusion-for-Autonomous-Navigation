import time
import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

GRAVITY = 9.801

i2c = board.I2C()

tca = adafruit_tca9548a.TCA9548A(i2c)

imu0 = LSM6DSOX(tca[1])
imu1 = LSM6DSOX(tca[4])
imu2 = LSM6DSOX(tca[7])


def collect_data(imu, axis):
    num_samples = 100
    data_average = 0.0
    timestep = 0.01
    for i in range(num_samples):
        data_average += imu.acceleration[axis]
        time.sleep(timestep)

    return data_average / num_samples


def format_calibration_data(data):
    bias_offset_x = -1.0 * data[0][1]
    bias_offset_y = -1.0 * data[1][1]
    bias_offset_z = -1.0 * data[2][1]

    scale_factor_x = (2.0 * GRAVITY) / (data[0][0] - data[0][2])
    scale_factor_y = (2.0 * GRAVITY) / (data[1][0] - data[1][2])
    scale_factor_z = (2.0 * GRAVITY) / (data[2][0] - data[2][2])

    return f"[[{bias_offset_x}, {scale_factor_x}], [{bias_offset_y}, {scale_factor_y}], [{bias_offset_z}, {scale_factor_z}]]"


imu0_data = [[0] * 3] * 3
imu1_data = [[0] * 3] * 3
imu2_data = [[0] * 3] * 3

input("Orient system upright and press enter to continue")
print("Collecting data")

imu0_data[0][1] = collect_data(imu0, 0)
imu0_data[1][1] = collect_data(imu0, 1)
imu0_data[2][0] = collect_data(imu0, 2)

imu1_data[0][1] = collect_data(imu1, 0)
imu1_data[1][1] = collect_data(imu1, 1)
imu1_data[2][0] = collect_data(imu1, 2)

imu2_data[0][1] = collect_data(imu2, 0)
imu2_data[1][1] = collect_data(imu2, 1)
imu2_data[2][0] = collect_data(imu2, 2)

input("Orient system with X-axis up and press enter to continue")

imu0_data[0][0] = collect_data(imu0, 0)
imu0_data[2][1] = collect_data(imu0, 2)

imu1_data[0][0] = collect_data(imu1, 0)
imu1_data[2][1] = collect_data(imu1, 2)

imu2_data[0][0] = collect_data(imu2, 0)
imu2_data[2][1] = collect_data(imu2, 2)

input("Orient system with Z-axis down and press enter to continue")

imu0_data[2][2] = collect_data(imu0, 2)
imu1_data[2][2] = collect_data(imu1, 2)
imu2_data[2][2] = collect_data(imu2, 2)

input("Orient system with X-axis down and press enter to continue")

imu0_data[0][2] = collect_data(imu0, 0)
imu1_data[0][2] = collect_data(imu1, 0)
imu2_data[0][2] = collect_data(imu2, 0)

input("Orient system with Y-axis up and press enter to continue")

imu0_data[1][0] = collect_data(imu0, 1)
imu1_data[1][0] = collect_data(imu1, 1)
imu2_data[1][0] = collect_data(imu2, 1)

input("Orient system with Y-axis down and press enter to continue")

imu0_data[1][2] = collect_data(imu0, 1)
imu1_data[1][2] = collect_data(imu1, 1)
imu2_data[1][2] = collect_data(imu2, 1)

print("Calibration complete")

print("IMU0 Data:", imu0_data)
print("IMU1 Data:", imu1_data)
print("IMU2 Data:", imu2_data)

print("Paste this in for the calibration matrix:")
print(f"ACM = [{format_calibration_data(imu0_data)}, {format_calibration_data(imu1_data)}, {format_calibration_data(imu2_data)}]")
