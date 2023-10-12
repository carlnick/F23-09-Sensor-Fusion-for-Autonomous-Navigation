import board
import adafruit_tca9548a
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX


class IMU:
    def __init__(self, imu0_port: int, imu1_port: int, imu2_port: int):
        i2c = board.I2C()
        tca = adafruit_tca9548a.TCA9548A(i2c)
        self.imu0 = LSM6DSOX(tca[imu0_port])
        self.imu1 = LSM6DSOX(tca[imu1_port])
        self.imu2 = LSM6DSOX(tca[imu2_port])
        self.accelerometer_calibration_matrix = [[[0.0] * 2] * 3] * 3
        self.gyroscope_calibration_matrix = [[[0.0] * 2] * 3] * 3

    def _vote(self, a: float, b: float, c: float) -> float:
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

        # Return the value pair corresponding to the minimum difference
        return diff_pair_mapping[min_diff]

    def get_acceleration(self, axis: str) -> float:
        axis_num = ord(axis) - ord('x')

        return self._vote(self.imu0.acceleration[axis_num],
                          self.imu1.acceleration[axis_num],
                          self.imu2.acceleration[axis_num])

    def get_gyro(self, axis: str) -> float:
        axis_num = ord(axis) - ord('x')
        return self._vote(self.imu0.gyro[axis_num],
                          self.imu1.gyro[axis_num],
                          self.imu2.gyro[axis_num])

    def get_acceleration_all(self) -> list:
        return [self.get_acceleration(axis) for axis in "xyz"]

    def get_gyro_all(self) -> list:
        return [self.get_gyro(axis) for axis in "xyz"]
