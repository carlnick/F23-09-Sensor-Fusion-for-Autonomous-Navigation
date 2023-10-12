import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from typing import Literal

# ACM is the accelerometer calibration matrix, to be filled in with the calibration data
ACM = [[[0.0, 1.0]] * 3] * 3

# GCM is the gyroscope calibration matrix, to be filled in with the calibration data
GCM = [[[0.0, 1.0]] * 3] * 3

# Multiplexer port numbers
allowed_ports = Literal[0, 1, 2, 3, 4, 5, 6, 7]


def _vote_and_average(a: float, b: float, c: float) -> float:
    # Calculate absolute differences
    diff_ab = abs(a - b)
    diff_ac = abs(a - c)
    diff_bc = abs(b - c)

    # Find the minimum difference
    min_diff = min(diff_ab, diff_ac, diff_bc)

    # Map differences to value pairs
    diff_pair_mapping = {
        diff_ab: (a, b),
        diff_ac: (a, c),
        diff_bc: (b, c)
    }

    # Return the average of the value pair corresponding to the minimum difference
    best_values = diff_pair_mapping[min_diff]
    return (best_values[0] + best_values[1]) / 2.0


class IMU:
    def __init__(self, imu_ports: list[allowed_ports]):
        """

        :param imu_ports: A list of ports that the IMUs are connected to, e.g. [2, 5, 6]
        """
        mux = adafruit_tca9548a.TCA9548A(board.I2C())

        self.IMU0 = LSM6DSOX(mux[imu_ports[0]])
        self.IMU1 = LSM6DSOX(mux[imu_ports[1]])
        self.IMU2 = LSM6DSOX(mux[imu_ports[2]])

    def _get_sensor_data(self, axis: str, raw_data, CM) -> float:
        """


        :param axis: The axis to get data from
        :param raw_data: The raw data function that gets the data from the sensor and axis
        :param CM: The calibration matrix of the sensor
        :return: A calibrated, filtered, and averaged value from the sensor and axis
        """

        # Check that the axis is valid
        if len(axis) != 1:
            raise ValueError("Axis must be a single character (x, y, or z)")

        axis_number = ord(axis) - ord('x')

        if axis_number < 0 or axis_number > 2:
            raise ValueError("Axis must be x, y, or z")

        # Get the raw data as a list
        raw_data = [raw_data(self.IMU0, axis_number),
                    raw_data(self.IMU1, axis_number),
                    raw_data(self.IMU2, axis_number)]

        # f(x) = (x + b) * s
        # x is the raw data, b is the bias offset, and s is the scale factor
        # Apply the calibration matrix to the raw data
        calibrated_data = [(raw_data[0] + CM[0][axis_number][0]) * CM[0][axis_number][1],
                           (raw_data[1] + CM[1][axis_number][0]) * CM[1][axis_number][1],
                           (raw_data[2] + CM[2][axis_number][0]) * CM[2][axis_number][1]]

        return _vote_and_average(calibrated_data[0], calibrated_data[1], calibrated_data[2])

    def get_acceleration(self, axis: str) -> float:
        """
        Uses a lambda function to get the raw data from the sensor and axis and passes it to the _get_sensor_data
        method to do the calibration, filtering, and averaging

        :param axis: The axis to get data from
        :return: The calibrated, filtered, and averaged linear acceleration in meters per second^2 in the selected axis
        """
        return self._get_sensor_data(axis, lambda imu, axis_num: imu.acceleration[axis_num], ACM)

    def get_gyroscope(self, axis: str) -> float:
        """
        Uses a lambda function to get the raw data from the sensor and axis and passes it to the _get_sensor_data
        method to do the calibration, filtering, and averaging

        :param axis: The axis to get data from
        :return: The calibrated, filtered, and averaged angular velocity in degrees per second in the selected axis
        """
        return self._get_sensor_data(axis, lambda imu, axis_num: imu.gyro[axis_num], GCM)

    def get_acceleration_all(self) -> list:
        """
        Gets the acceleration in all axes
        :return: A list containing the acceleration in the x, y, and z axes in meters per second^2, in that order
        """
        return [self.get_acceleration(axis) for axis in 'xyz']

    def get_gyroscope_all(self) -> list:
        """
        Gets the angular velocity in all axes
        :return: A list containing the angular velocity in the x, y, and z axes in degrees per second, in that order
        """
        return [self.get_gyroscope(axis) for axis in 'xyz']
