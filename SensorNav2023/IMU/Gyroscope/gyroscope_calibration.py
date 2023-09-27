# from pynput import keyboard
# import numpy as np
# import time
# import board
# import adafruit_tca9548a
# from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
#
# i2c = board.I2C()
#
# tca = adafruit_tca9548a.TCA9548A(i2c)
#
# imu0 = LSM6DSOX(tca[0])
# imu1 = LSM6DSOX(tca[3])
# imu2 = LSM6DSOX(tca[5])
#
#
# def on_key_release(key):
#     return key == keyboard.Key.enter
#
#
# def collect_rotate_integrate(imu, axis):
#     data = []
#     timestep = 0.1
#
#     with keyboard.Listener(on_release=on_key_release) as listener:
#         listener.join()
#
#     while True:
#         data.append(imu.gyro[axis])
#         time.sleep(timestep)
#
#     # integrate data using cumtrapz
#     data_integrated = np.cumtrapz(data, dx=timestep)
#     return 360.0 / data_integrated
#
#
# input("Orient system upright, press enter, rotate 360 degrees, and press enter again")
# print("Collecting data")
# print(collect_rotate_integrate(imu0, 0))

import keyboard
import time
import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import threading


def initialize_sensors():
    i2c = board.I2C()
    tca = adafruit_tca9548a.TCA9548A(i2c)
    imu0 = LSM6DSOX(tca[0])
    imu1 = LSM6DSOX(tca[3])
    imu2 = LSM6DSOX(tca[5])
    return imu0, imu1, imu2


def collect_data(imu, axis):
    raw_data = []
    timestep = 0.01
    while True:
        if keyboard.is_pressed("e"):
            break
        raw_data.append(imu.gyro[axis])
        time.sleep(timestep)

    return raw_data


if __name__ == "__main__":
    imu0, imu1, imu2 = initialize_sensors()
    try:
        print("press e, rotate 360 around X-axis, and press e again to stop data collection")
        thread1 = threading.Thread(target=collect_data, args=(imu0, 0))
        thread2 = threading.Thread(target=collect_data, args=(imu1, 0))
        thread3 = threading.Thread(target=collect_data, args=(imu2, 0))
        thread1.start()
        thread2.start()
        thread3.start()
    except KeyboardInterrupt:
        print("Keyboard interrupt detected. Exiting the program.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")

    keyboard.unhook_all()
