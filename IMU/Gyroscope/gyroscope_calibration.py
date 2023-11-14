""" IMPORT STATEMENTS """
from time import sleep
from typing import Literal
from board import I2C
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from numpy import trapz, rad2deg
from pynput.keyboard import Key, Listener

TIMESTEP = 0.01
NUM_ROTATIONS = 5
ALLOWED_PORTS = Literal[0, 1, 2, 3, 4, 5, 6, 7]


def initialize_sensors(imu_ports: list[ALLOWED_PORTS]) -> list[LSM6DSOX]:
    multiplexer = TCA9548A(I2C())
    sensors = []
    for port in imu_ports:
        sensors.append(LSM6DSOX(multiplexer[port]))

    return sensors


def get_bias_offset(sensor: LSM6DSOX) -> list[float]:
    num_samples = 100
    bias_offsets = [0.0, 0.0, 0.0]
    for i in range(num_samples):
        bias_offsets[0] += sensor.gyro[0]
        bias_offsets[1] += sensor.gyro[1]
        bias_offsets[2] += sensor.gyro[2]
        sleep(TIMESTEP)

    bias_offsets[0] /= -1.0 * float(num_samples)
    bias_offsets[1] /= -1.0 * float(num_samples)
    bias_offsets[2] /= -1.0 * float(num_samples)

    return bias_offsets


def get_bias_offset_all(sensors: list[LSM6DSOX]) -> list[list[float]]:
    print("Keep system still and press enter to start calibration.")
    wait_until_ready()
    print("Getting bias offsets...")
    bias_offsets = []
    for sensor in sensors:
        bias_offsets.append(get_bias_offset(sensor))

    return bias_offsets


def on_key_press(key):
    if key == Key.enter:
        print("ENTER PRESSED")
        return False


def get_scale_factors(sensors: list[LSM6DSOX], axis: int):
    data = [[], [], []]

    with Listener(on_press=on_key_press) as listener:
        while True:
            if not listener.running:
                break
            data[0].append(sensors[0].gyro[axis])
            data[1].append(sensors[1].gyro[axis])
            data[2].append(sensors[2].gyro[axis])
            sleep(TIMESTEP)

    scale_factors = [0.0, 0.0, 0.0]
    scale_factors[0] += float(NUM_ROTATIONS * 360.0) / rad2deg(trapz(data[0], dx=TIMESTEP))
    scale_factors[1] += float(NUM_ROTATIONS * 360.0) / rad2deg(trapz(data[1], dx=TIMESTEP))
    scale_factors[2] += float(NUM_ROTATIONS * 360.0) / rad2deg(trapz(data[2], dx=TIMESTEP))

    return scale_factors


def wait_until_ready():
    with Listener(on_press=on_key_press) as listener:
        listener.join()


def get_scale_factors_all(sensors: list[LSM6DSOX]) -> list[list[float]]:
    scale_factors = []
    print("Prepare to rotate the system about the x-axis, press enter when ready.")
    wait_until_ready()
    print(f"Collecting data, press enter after {NUM_ROTATIONS} rotations.")
    scale_factors.append(get_scale_factors(sensors, 0))
    print("Prepare to rotate the system about the y-axis, press enter when ready.")
    wait_until_ready()
    print(f"Collecting data, press enter after {NUM_ROTATIONS} rotations.")
    scale_factors.append(get_scale_factors(sensors, 1))
    print("Prepare to rotate the system about the z-axis, press enter when ready.")
    wait_until_ready()
    print(f"Collecting data, press enter after {NUM_ROTATIONS} rotations.")
    scale_factors.append(get_scale_factors(sensors, 2))
    print("Data collection complete.")
    return scale_factors


def format_calibration_data(bias_offsets: list[float], scale_factors: list[float]) -> str:
    return f"[[{bias_offsets[0]}, {scale_factors[0]}], [{bias_offsets[1]}, {scale_factors[1]}], [{bias_offsets[2]}, {scale_factors[2]}]]"


def format_calibration_data_all(bias_offsets: list[list[float]], scale_factors: list[list[float]]):
    imu_0_line = format_calibration_data(bias_offsets[0], scale_factors[0])
    imu_1_line = format_calibration_data(bias_offsets[1], scale_factors[1])
    imu_2_line = format_calibration_data(bias_offsets[2], scale_factors[2])

    return f"GCM = [{imu_0_line}, {imu_1_line}, {imu_2_line}]"


if __name__ == "__main__":
    imus = initialize_sensors([1, 4, 7])
    bias_offsets = get_bias_offset_all(imus)
    scale_factors = get_scale_factors_all(imus)
    print("Calibration complete.")
    print("Paste this in for the calibration matrix:")
    print(format_calibration_data_all(bias_offsets, scale_factors))
