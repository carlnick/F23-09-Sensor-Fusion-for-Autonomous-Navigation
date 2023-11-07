from numpy import array, dot, zeros
from board import I2C
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from typing import Literal

# ACM is the accelerometer calibration matrix, to be filled in with the calibration data
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
GCM = [[0.006119647956267721, 0.016819258035925055, -0.001092074329588504], [-0.000989448969571235, -0.0026483298820527243, 0.0018803195613282646], [-0.003199406685062105, -0.003317150959724769, 0.003518965562794444]]

# Multiplexer port numbers
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


def _calibrate_accel(raw_data: list[list[float]]) -> list[list[float]]:
    calibrated_data: list[list[float]] = [[0.0] * 3] * 3
    for imu_index, data in enumerate(raw_data):
        calibration_matrix = array(ACM[imu_index])
        calibrated_data[imu_index] = dot(data + [1.0], calibration_matrix)

    return calibrated_data


def _single_vote_and_average(data: list[float]) -> float:
    diff_0 = abs(data[0] - data[1])
    diff_1 = abs(data[0] - data[2])
    diff_2 = abs(data[1] - data[2])

    min_diff = min(diff_0, diff_1, diff_2)

    diff_pair_mapping = {
        diff_0: (data[0], data[1]),
        diff_1: (data[0], data[2]),
        diff_2: (data[1], data[2])
    }

    best_values = diff_pair_mapping[min_diff]
    return (best_values[0] + best_values[1]) / 2.0


def _vote_and_average(calibrated_data: list[list[float]]) -> list[float]:
    voted_data: list[float] = []
    for data in calibrated_data:
        voted_data.append(_single_vote_and_average(data))

    return voted_data


def _calibrate_gyro(raw_data: list[list[float]]) -> list[list[float]]:
    raw_data_copy = array(raw_data)
    calibration_matrix = array(GCM)
    return raw_data_copy + calibration_matrix


class IMU:
    def __init__(self, imu_ports: list[ALLOWED_PORTS]) -> None:
        self.IMUs = init_imus(imu_ports)

    def get_acceleration(self):
        raw_data = [list(self.IMUs[0].acceleration), list(self.IMUs[1].acceleration), list(self.IMUs[2].acceleration)]
        calibrated_data = _calibrate_accel(raw_data)
        return _vote_and_average(calibrated_data)

    def get_gyroscope(self):
        raw_data = list([self.IMUs[0].gyro, self.IMUs[1].gyro, self.IMUs[2].gyro])
        calibrated_data = _calibrate_gyro(raw_data)
        return _vote_and_average(calibrated_data)
