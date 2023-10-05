import numpy as np
import board
import adafruit_mmc56x3 as magnetometer
import adafruit_tca9548a as multiplexer


# class for magnetometer subsystem
class Magnetometer:

    # Magnetometer 0 calibration parameters
    hard_iron0 = np.array([33.99986952, 5.06027207, -127.74162784])
    soft_iron0 = [[1.1568729, 0.02053911, 0.02432764],
                [0.02053911, 1.17281592, 0.01611224],
                [0.02432764, 0.01611224 ,1.18665655]]


    # Magnetometer 1 calibration parameters
    hard_iron1 = np.array([-43.09198574, 11.00891923, 75.1944151])
    soft_iron1 = [[1.22511675, 0.02326508, 0.02639732],
                [0.02326508, 1.2591332,  0.02921156],
                [0.02639732, 0.02921156, 1.24770114]]

    # Magnetometer 2 calibration parameters
    hard_iron2 = np.array([18.40675578, -10.6142152, -33.00257546])
    soft_iron2 = [[ 1.12147563,  0.02356025,  0.01364192],
                [ 0.02356025,  1.13388023, -0.00540468],
                [ 0.01364192, -0.00540468,  1.1400925 ]]


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
    def get_magnetic():
	
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
    def get_heading(mag_data):
        heading = -1* (np.arctan2(mag_data[0], mag_data[1]) * 180) / np.pi
        if(heading < 0):
            heading += 360

        return heading
        
while(1):
    mag_data = Magnetometer.get_magnetic()
    print(Magnetometer.get_heading(mag_data))