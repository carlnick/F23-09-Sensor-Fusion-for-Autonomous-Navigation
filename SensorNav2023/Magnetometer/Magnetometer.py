import numpy as np
import board
import adafruit_mmc56x3 as magnetometer
import adafruit_tca9548a as multiplexer


# class for magnetometer subsystem
class Magnetometer:

    # Magnetometer 0 calibration parameters
    hard_iron0 = np.array([59.39487654, 5.39098516, -7.68357777])
    soft_iron0 = [[1.14979637, 0.00569657, 0.00237638],
                [0.00569657, 1.1964999, -0.01135455],
                [0.00237638, -0.01135455, 1.19870142]]



    # Magnetometer 1 calibration parameters
    hard_iron1 = np.array([-167.74038797, 24.09046639, -96.88056222])
    soft_iron1 = [[1.11254552, 0.02645651, 0.02254459],
                [0.02645651, 1.14822925, 0.00176098],
                [0.02254459, 0.00176098, 1.0770171]]


    # Magnetometer 2 calibration parameters
    hard_iron2 = np.array([15.41402626, -27.23083942, -14.3872628])
    soft_iron2 = [[1.14568407e+00, 6.48283959e-04, 7.33334573e-05],
                [6.48283959e-04,  1.15233447e+00, -1.16823200e-03],
                [7.33334573e-05, -1.16823200e-03, 1.20095746e+00]]


    # voting function
    def __mag_voting(mag_one, mag_two, mag_three, threshold):
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
    def __apply_calibration(mag, hard_iron, soft_iron):
        return np.matmul(soft_iron,  (mag - hard_iron).reshape(3,1))


    # main function to get calibrated and filtered magnetic field data
    def get_magnetic(self):
	
        i2c = board.I2C()
        mux = multiplexer.TCA9548A(i2c)
        
        # get raw data
        mag_0 = np.asarray(magnetometer.MMC5603(mux[0]).magnetic)
        mag_1 = np.asarray(magnetometer.MMC5603(mux[3]).magnetic)
        mag_2 = np.asarray(magnetometer.MMC5603(mux[6]).magnetic)

        # calibrate raw data
        cal_mag0 = Magnetometer.__apply_calibration(mag_0, Magnetometer.hard_iron0, Magnetometer.soft_iron0)
        cal_mag1 = Magnetometer.__apply_calibration(mag_1, Magnetometer.hard_iron1, Magnetometer.soft_iron1)
        cal_mag2 = Magnetometer.__apply_calibration(mag_2, Magnetometer.hard_iron2, Magnetometer.soft_iron2)
        
        threshold = 7.0
        
        return Magnetometer.__mag_voting(cal_mag0, cal_mag1, cal_mag2, threshold)
    
    # get the heading of the magnetic field data (x is the heading/north)
    def get_heading(self, mag_data, axis1, axis2):
        heading = -1* (np.arctan2(mag_data[axis1], mag_data[axis2]) * 180) / np.pi
        if(heading < 0):
            heading += 360

        return heading
        
if __name__ == "__main__":
    mag = Magnetometer()
    while(1):
        mag_data = mag.get_magnetic()
        print(mag.get_heading(mag_data, 0, 1))
