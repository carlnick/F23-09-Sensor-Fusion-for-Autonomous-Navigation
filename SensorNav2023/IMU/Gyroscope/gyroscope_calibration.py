from pynput import keyboard
import time
import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import numpy as np

timestep = 0.01


def initialize_sensors():
    i2c = board.I2C()
    tca = adafruit_tca9548a.TCA9548A(i2c)
    imu0 = LSM6DSOX(tca[1])
    imu1 = LSM6DSOX(tca[4])
    imu2 = LSM6DSOX(tca[7])
    return imu0, imu1, imu2


def on_key_press(key):
    print("ENTER PRESSED")
    if key == keyboard.Key.enter:
        return False


def collect_data(axis, imus):
    imu0_data = []
    imu1_data = []
    imu2_data = []

    with keyboard.Listener(on_press=on_key_press) as listener:
        while True:
            if not listener.running:
                break
            imu0_data.append(imus[0].gyro[axis])
            imu1_data.append(imus[1].gyro[axis])
            imu2_data.append(imus[2].gyro[axis])
            time.sleep(timestep)

    return [imu0_data, imu1_data, imu2_data]


if __name__ == "__main__":

    sensors = initialize_sensors()

    print("Prepare to start rotating around the x-axis")
    time.sleep(10)
    print("Start rotating")

    for axis in range(0, 3):
        data = collect_data(axis, sensors)
        imu0_rotation = np.rad2deg(np.trapz(data[0], dx=timestep))
        imu1_rotation = np.rad2deg(np.trapz(data[1], dx=timestep))
        imu2_rotation = np.rad2deg(np.trapz(data[2], dx=timestep))

        print(f"IMU 0 rotation in axis {axis}: {imu0_rotation} degrees")
        print(f"IMU 1 rotation in axis {axis}: {imu1_rotation} degrees")
        print(f"IMU 2 rotation in axis {axis}: {imu2_rotation} degrees")
        if axis == 0:
            print("Prepare to start rotating around the y-axis")
        elif axis == 1:
            print("Prepare to start rotating around the z-axis")
        else:
            break
        time.sleep(10)
        print("Start rotating")
