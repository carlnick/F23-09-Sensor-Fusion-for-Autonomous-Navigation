import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from typing import Literal
import numpy

# ACM is the accelerometer calibration matrix, to be filled in with the calibration data
# ACM = [[[0.0, 1.0]] * 3] * 3
# ACM = [[[-0.011712866627000003, 1.9786003080074333], [-0.011712866627000003, 1.9786003080074333], [-0.011712866627000003, 1.9786003080074333]], [[-0.07282555583100003, 2.0102546695712573], [-0.07282555583100003, 2.0102546695712573], [-0.07282555583100003, 2.0102546695712573]], [[-0.19314864027200002, 1.9854361392673074], [-0.19314864027200002, 1.9854361392673074], [-0.19314864027200002, 1.9854361392673074]]]
ACM = [[[2.03060745e-01, -9.60414554e-04, -1.09979863e-04],
        [-4.16799534e-04, 2.01761957e-01, -1.55430872e-03],
        [-8.71342361e-04, 4.43829649e-04, 2.04868707e-01],
        [-7.79055134e-03, 1.55764881e-03, 5.44368662e-04]], [[0.20397773, -0.00099193, -0.00063006],
                                                             [-0.00020754, 0.20499526, -0.00282797],
                                                             [-0.00037286, 0.00185254, 0.20366316],
                                                             [0.00224264, 0.01620165, -0.0207113]],
       [[2.02042694e-01, 1.01128445e-03, -3.63626995e-05],
        [-2.07620347e-03, 2.02551069e-01, -3.52552755e-03],
        [-5.49855458e-04, 2.47868362e-03, 2.02680258e-01],
        [-1.89117055e-03, 9.74183122e-03, -1.87680089e-02]]]

# GCM is the gyroscope calibration matrix, to be filled in with the calibration data
# GCM = [[[0.0, 1.0]] * 3] * 3
GCM = [[[0.005629123169994712, 1.7699775413818446], [0.015204435778748603, 1.7560386051917058],
        [0.0022067506729903296, 1.7725429987542713]],
       [[-0.0012217304763960301, 1.7753325202433676], [-0.0033872477458079973, 1.782776109740368],
        [0.004124867520932099, 1.7715026669880383]],
       [[-0.0029962939933612666, 1.7541433597049298], [-0.00312457569338285, 1.7759366905208875],
        [-0.005986479334340555, 1.759203832234968]]]

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


def _calibrate_accel(raw_data: list[float], imu_index: int):
    calibration_matrix = numpy.array(ACM[imu_index])
    raw_data.append(1)
    return numpy.dot(raw_data, calibration_matrix)


def _calibrate_gyro(raw_data: list[float], imu_index: int):
    # calibration_matrix = numpy.array(GCM[imu_index])
    # return numpy.dot(raw_data, calibration_matrix)
    return raw_data


class IMU:
    def __init__(self, imu_ports: list[allowed_ports]):
        """

        :param imu_ports: A list of ports that the IMUs are connected to, e.g. [2, 5, 6]
        """
        mux = adafruit_tca9548a.TCA9548A(board.I2C())

        self.IMU0 = LSM6DSOX(mux[imu_ports[0]])
        self.IMU1 = LSM6DSOX(mux[imu_ports[1]])
        self.IMU2 = LSM6DSOX(mux[imu_ports[2]])

    def get_acceleration(self, axis: str) -> float:
        """
        Uses a lambda function to get the raw data from the sensor and axis and passes it to the _get_sensor_data
        method to do the calibration, filtering, and averaging

        :param axis: The axis to get data from
        :return: The calibrated, filtered, and averaged linear acceleration in meters per second^2 in the selected axis
        """

        # Check that the axis is valid
        if len(axis) != 1:
            raise ValueError("Axis must be a single character (x, y, or z)")

        axis_number = ord(axis) - ord('x')

        if axis_number < 0 or axis_number > 2:
            raise ValueError("Axis must be x, y, or z")

        imu0_raw = [self.IMU0.acceleration[0], self.IMU0.acceleration[1], self.IMU0.acceleration[2]]
        imu1_raw = [self.IMU1.acceleration[0], self.IMU1.acceleration[1], self.IMU1.acceleration[2]]
        imu2_raw = [self.IMU2.acceleration[0], self.IMU2.acceleration[1], self.IMU2.acceleration[2]]

        imu0_cal = _calibrate_accel(imu0_raw, 0)
        imu1_cal = _calibrate_accel(imu1_raw, 1)
        imu2_cal = _calibrate_accel(imu2_raw, 2)

        return _vote_and_average(imu0_cal[axis_number], imu1_cal[axis_number], imu2_cal[axis_number])

    def get_gyroscope(self, axis: str) -> float:
        """
        Uses a lambda function to get the raw data from the sensor and axis and passes it to the _get_sensor_data
        method to do the calibration, filtering, and averaging

        :param axis: The axis to get data from
        :return: The calibrated, filtered, and averaged angular velocity in degrees per second in the selected axis
        """

        # Check that the axis is valid
        if len(axis) != 1:
            raise ValueError("Axis must be a single character (x, y, or z)")

        axis_number = ord(axis) - ord('x')

        if axis_number < 0 or axis_number > 2:
            raise ValueError("Axis must be x, y, or z")

        imu0_raw = [self.IMU0.gyro[0], self.IMU0.gyro[1], self.IMU0.gyro[2]]
        imu1_raw = [self.IMU1.gyro[0], self.IMU1.gyro[1], self.IMU1.gyro[2]]
        imu2_raw = [self.IMU2.gyro[0], self.IMU2.gyro[1], self.IMU2.gyro[2]]

        imu0_cal = _calibrate_gyro(imu0_raw, 0)
        imu1_cal = _calibrate_gyro(imu1_raw, 1)
        imu2_cal = _calibrate_gyro(imu2_raw, 2)

        return _vote_and_average(imu0_cal[axis_number], imu1_cal[axis_number], imu2_cal[axis_number])



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
