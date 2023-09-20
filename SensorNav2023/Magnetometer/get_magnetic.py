import numpy as np
import board
import adafruit_mmc56x3 as magnetometer
import adafruit_tca9548a as multiplexer
with mag_voting import mag_voting

# sensor indexes
X = 0
Y = 1
Z = 2


# Magnetometer 0 calibration parameters
hard_iron0 = np.zeros([3, 1])
soft_iron0 = np.eye(3)

# Magnetometer 1 calibration parameters
hard_iron1 = np.zeros([3, 1])
soft_iron1 = np.eye(3)

# Magnetometer 2 calibration parameters
hard_iron2 = np.zeros([3, 1])
soft_iron2 = np.eye(3)

def apply_calibration(mag, hard_iron, soft_iron):
	return soft_iron @ (mag - hard_iron)


def get_magnetic():
	
	i2c = board.I2C()
	mux = multiplexer.TCA9548A(i2c)
	
	# get raw data
    mag_0 = magnetometer.MMC5603(mux[0])
    mag_1 = magnetometer.MMC5603(mux[1])
    mag_2 = magnetometer.MMC5603(mux[2])

	# calibrate raw data
	cal_mag0 = apply_calibration(mag_0, hard_iron0, soft_iron0)
	cal_mag1 = apply_calibration(mag_1, hard_iron1, soft_iron1)
	cal_mag2 = apply_calibration(mag_2, hard_iron2, soft_iron2)
	
	threshold = 7.0
	
	return(cal_mag0, cal_mag1, cal_mag2, threshold)

