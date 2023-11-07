from board import I2C
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import AccelRange, Rate, GyroRange
from typing import Literal
import numpy

ALLOWED_PORTS = Literal[0, 1, 2, 3, 4, 5, 6, 7]


def init_imus(imu_ports: list[ALLOWED_PORTS]) -> list[LSM6DSOX]:
    """
    Initializes the IMUs on the specified multiplexer ports and sets their measurement ranges and data rates
    :param imu_ports: A list of ports on the multiplexer that the IMUs are connected to, e.g. [1, 4, 7]
    :return: A list containing the initialized IMUs
    """
    mux = TCA9548A(I2C())

    imus: list[LSM6DSOX] = list()
    for port in imu_ports:
        imus.append(LSM6DSOX(mux[port]))

    for imu in imus:
        imu.accelerometer_range = AccelRange.RANGE_16G
        imu.accelerometer_data_rate = Rate.RATE_833_HZ
        imu.gyro_range = GyroRange.RANGE_125_DPS
        imu.gyro_data_rate = Rate.RATE_833_HZ

    return imus


def get_bias_offsets(imus: list[LSM6DSOX]) -> numpy.ndarray:
    input("Keep system still and press enter to start calibration.")
    print("Getting bias offsets...")
    bias_offsets = numpy.zeros((3, 3))
    num_samples = 1000

    for _ in range(num_samples):
        bias_offsets += [imus[0].gyro, imus[1].gyro, imus[2].gyro]

    bias_offsets /= num_samples * -1
    return bias_offsets


if __name__ == "__main__":
    imus = init_imus([1, 4, 7])
    print(get_bias_offsets(imus))
