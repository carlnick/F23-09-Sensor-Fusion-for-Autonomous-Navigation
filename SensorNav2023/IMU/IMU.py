from typing import Literal
from numpy import array, dot, rad2deg
from board import I2C
from adafruit_tca9548a import TCA9548A
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from typing import Literal

ALLOWED_PORTS = Literal[0, 1, 2, 3, 4, 5, 6, 7]


def init_imus(imu_ports: list[ALLOWED_PORTS]) -> list[LSM6DSOX]:
    """
    Initializes the IMUs on the specified multiplexer ports and sets their measurement ranges and data rates
    :param imu_ports: A list of ports on the multiplexer that the IMUs are connected to, e.g. [1, 4, 7]
    :return: A list containing the initialized IMUs
    """
    mux = TCA9548A(I2C())

    IMUS: list[LSM6DSOX] = list()
    for port in imu_ports:
        IMUS.append(LSM6DSOX(mux[port]))

    for IMU in IMUS:
        IMU.accelerometer_range = AccelRange.RANGE_16G
        IMU.accelerometer_data_rate = Rate.RATE_833_HZ
        IMU.gyro_range = GyroRange.RANGE_125_DPS
        IMU.gyro_data_rate = Rate.RATE_833_HZ

    return IMUS


# ACM is the accelerometer calibration matrix, to be filled in with the calibration data
# ACM = [[[2.03060745e-01, -9.60414554e-04, -1.09979863e-04],
#         [-4.16799534e-04, 2.01761957e-01, -1.55430872e-03],
#         [-8.71342361e-04, 4.43829649e-04, 2.04868707e-01],
#         [-7.79055134e-03, 1.55764881e-03, 5.44368662e-04]], [[0.20397773, -0.00099193, -0.00063006],
#                                                              [-0.00020754, 0.20499526, -0.00282797],
#                                                              [-0.00037286, 0.00185254, 0.20366316],
#                                                              [0.00224264, 0.01620165, -0.0207113]],
#        [[2.02042694e-01, 1.01128445e-03, -3.63626995e-05],
#         [-2.07620347e-03, 2.02551069e-01, -3.52552755e-03],
#         [-5.49855458e-04, 2.47868362e-03, 2.02680258e-01],
#         [-1.89117055e-03, 9.74183122e-03, -1.87680089e-02]]]
ACM = [[[1.98928408e+00, -1.64728654e-03, -3.07313158e-03],
        [-2.34640647e-03, 1.97635695e+00, 5.20865679e-03],
        [1.11118498e-02, -5.41427323e-03, 2.00689841e+00],
        [-4.72902976e-02, 4.22600855e-02, -9.40499565e-03]], [[1.99869805e+00, -1.28021315e-03, -2.19721620e-03],
                                                              [-1.42593742e-03, 2.00812688e+00, -1.58527151e-02],
                                                              [8.51527134e-03, 1.60442458e-02, 1.99418627e+00],
                                                              [5.77683173e-02, 1.63253112e-01, -2.06402525e-01]],
       [[1.97934249, 0.01758001, 0.00403731],
        [-0.01858618, 1.9843145, -0.01042022],
        [0.00892415, 0.01038862, 1.9858985],
        [0.01102197, 0.09660815, -0.19623267]]]

# GCM is the gyroscope calibration matrix, to be filled in with the calibration data
GCM = [[0.006093304392870428, 0.015943430000658662, -0.0015018885462645903],
       [-0.00017295122056481318, -0.0028619799991124735, 0.002532265486795099],
       [-0.002850831708515362, -0.003336545931037562, 0.0031959705680972388]]

# Multiplexer port numbers
allowed_ports = Literal[0, 1, 2, 3, 4, 5, 6, 7]


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
    for axis in range(3):
        axis_data = [calibrated_data[0][axis], calibrated_data[1][axis], calibrated_data[2][axis]]
        voted_data.append(_single_vote_and_average(axis_data))

    return voted_data


def _calibrate_gyro(raw_data: list[list[float]]) -> list[list[float]]:
    raw_data_array = array(raw_data)
    calibration_matrix = array(GCM)
    return raw_data_array + calibration_matrix


class IMU:
    def __init__(self, imu_ports: list[allowed_ports]) -> None:
        self.IMUs = init_imus(imu_ports)

    def get_acceleration(self):
        raw_data = [list(self.IMUs[0].acceleration), list(self.IMUs[1].acceleration), list(self.IMUs[2].acceleration)]
        calibrated_data = _calibrate_accel(raw_data)
        return _vote_and_average(calibrated_data)

    def get_gyroscope(self):
        raw_data = list([self.IMUs[0].gyro, self.IMUs[1].gyro, self.IMUs[2].gyro])
        calibrated_data = _calibrate_gyro(raw_data)
        return _vote_and_average(calibrated_data)
