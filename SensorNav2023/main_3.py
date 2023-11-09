from SensorFusion.comp_filter_3 import ComplementaryFilter
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer

if __name__ == '__main__':
    imu = IMU([1, 4, 7])
    mag = Magnetometer()
    filter = ComplementaryFilter(imu.get_acceleration(), mag.get_magnetic())

    while True:
        print(filter.iterate(imu.get_acceleration(), imu.get_gyroscope(), mag.get_magnetic()))
