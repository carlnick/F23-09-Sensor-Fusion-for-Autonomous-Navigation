""" IMPORT STATEMENTS """
import board
import adafruit_mmc56x3 as magnetometer
import adafruit_tca9548a as multiplexer
import numpy as np


# class for magnetometer subsystem
class Magnetometer:
    # Magnetometer 0 calibration parameters
    hard_iron0 = np.array([ -0.52982801, -16.21490222, 50.71148091])
    soft_iron0 =  [[ 8.44547481e-01, -2.26801682e-04, 8.30002799e-03],
 [-2.26801682e-04, 9.04895957e-01, -5.19969901e-04],
 [ 8.30002799e-03, -5.19969901e-04, 9.12739762e-01]]


    # Magnetometer 1 calibration parameters
    hard_iron1 = np.array([139.36890564, -20.49838478, -54.54741062])
    soft_iron1 =  [[ 0.81522424, 0.02080285, 0.02185324],
 [ 0.02080285, 0.81675312, -0.01092806],
 [ 0.02185324, -0.01092806, 0.75557405]]


    # Magnetometer 2 calibration parameters
    hard_iron2 = np.array([ -2.62034814, 23.97796546, -33.37365645])
    soft_iron2 =  [[0.83060381, 0.00174827, 0.00215527],
 [0.00174827, 0.86318189, 0.0049377 ],
 [0.00215527, 0.0049377, 0.88446374]]


    def __init__(self):
        mux = multiplexer.TCA9548A(board.I2C())
        self.mag0 = magnetometer.MMC5603(mux[0])
        self.mag1 = magnetometer.MMC5603(mux[3])
        self.mag2 = magnetometer.MMC5603(mux[6])

        self.mag0.continuous_mode = True
        self.mag1.continuous_mode = True
        self.mag2.continuous_mode = True

        self.mag0.data_rate = 1000
        self.mag1.data_rate = 1000
        self.mag1.data_rate = 1000

    # voting function
    def _mag_voting(self, mag_one, mag_two, mag_three, threshold):
        # Convert readings to NumPy arrays for efficient calculations
        x_arr = np.array([mag_one[0], mag_two[0], mag_three[0]])
        y_arr = np.array([mag_one[1], mag_two[1], mag_three[1]])
        z_arr = np.array([mag_one[2], mag_two[2], mag_three[2]])

        # Calculate the median for each axis
        median_x = np.median(x_arr)
        median_y = np.median(y_arr)
        median_z = np.median(z_arr)

        # Find the largest outlier for each axis above the threshold
        max_outlier_x = x_arr[np.argmax(np.abs(x_arr - median_x))]
        max_outlier_y = y_arr[np.argmax(np.abs(y_arr - median_y))]
        max_outlier_z = z_arr[np.argmax(np.abs(z_arr - median_z))]

        # Check if the outliers are above the threshold
        if np.abs(max_outlier_x - median_x) > threshold:
            x_arr = x_arr[x_arr != max_outlier_x]
        if np.abs(max_outlier_y - median_y) > threshold:
            y_arr = y_arr[y_arr != max_outlier_y]
        if np.abs(max_outlier_z - median_z) > threshold:
            z_arr = z_arr[z_arr != max_outlier_z]

        # Calculate the average for each axis using the filtered values
        avg_x = np.mean(x_arr)
        avg_y = np.mean(y_arr)
        avg_z = np.mean(z_arr)

        return [avg_x, avg_y, avg_z]

    # applies hard and soft iron calibration
    def _apply_calibration(self, mag, hard_iron, soft_iron):
        return np.matmul(soft_iron, (mag - hard_iron).reshape(3, 1))

    # main function to get calibrated and filtered magnetic field data
    def get_magnetic(self):
        # get raw data
        mag_0 = np.asarray(self.mag0.magnetic)
        mag_1 = np.asarray(self.mag1.magnetic)
        mag_2 = np.asarray(self.mag2.magnetic)

        # calibrate raw data
        cal_mag0 = self._apply_calibration(mag_0, Magnetometer.hard_iron0, Magnetometer.soft_iron0)
        cal_mag1 = self._apply_calibration(mag_1, Magnetometer.hard_iron1, Magnetometer.soft_iron1)
        cal_mag2 = self._apply_calibration(mag_2, Magnetometer.hard_iron2, Magnetometer.soft_iron2)

        # print("Mag 0: ", cal_mag0[0], cal_mag0[1], cal_mag0[2])
        # print("Mag 1: ", cal_mag1[0], cal_mag1[1], cal_mag1[2])
        # print("Mag 2: ", cal_mag2[0], cal_mag2[1], cal_mag2[2])
        # print("__________________________________")

        threshold = 7.0

        return self._mag_voting(cal_mag0, cal_mag1, cal_mag2, threshold)

    # get the heading of the magnetic field data (x is the heading/north)
    def get_heading(self, mag_data, axis1, axis2):
        heading = (np.arctan2(mag_data[axis1], mag_data[axis2]) * 180) / np.pi

        if heading < 0:
            heading += 360

        return heading


if __name__ == "__main__":
    mag = Magnetometer()
    while 1:
        mag_data = mag.get_magnetic()
        print(mag.get_heading(mag_data, 0, 1))
