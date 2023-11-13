from board import I2C
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
from time import perf_counter


def init_imu() -> LSM6DSOX:
    imu = LSM6DSOX(I2C())
    imu.accelerometer_range = AccelRange.RANGE_16G
    imu.accelerometer_data_rate = Rate.RATE_1_66K_HZ
    imu.gyro_range = GyroRange.RANGE_125_DPS
    imu.gyro_data_rate = Rate.RATE_1_66K_HZ
    return imu


if __name__ == "__main__":
    imu = init_imu()

    num_samples = 0
    total_time = 0.0

    while True:
        start_time = perf_counter()
        print(imu.acceleration)
        end_time = perf_counter()

        total_time += end_time - start_time
        num_samples += 1

        avg_data_rate = float(num_samples / total_time)
        print(f"Average data rate: {round(avg_data_rate, 3)} Hz")
