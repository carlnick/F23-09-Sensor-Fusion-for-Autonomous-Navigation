# IMPORT STATEMENTS
from SensorFusion.Vector import Vector
from SensorFusion.ComplementaryFilter import ComplementaryFilter
from SensorFusion.SensorFusion import orientation, currentPositionVelocity 
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
from time import sleep, time
import numpy
		
if __name__ == "__main__":
	imu = IMU([1, 4, 7])
	mag = Magnetometer()
	
	prevTime = time()
	angularPos = Vector(0, 0, 0)
	
	vGyroscope = Vector()
		
	
	while(True):
		vGyroscope.x = numpy.rad2deg(imu.get_gyroscope('x'))
		vGyroscope.y = numpy.rad2deg(imu.get_gyroscope('y'))
		vGyroscope.z = numpy.rad2deg(imu.get_gyroscope('z'))
		
		currTime = time()
		deltaTime = currTime - prevTime
		result = Vector()
		result.x = angularPos.x + vGyroscope.x * deltaTime
		result.y = angularPos.y + vGyroscope.y * deltaTime
		result.z = angularPos.z + vGyroscope.z * deltaTime
		
		prevTime = currTime
		
		angularPos = result
		
		print(angularPos)
		
		sleep(1)
