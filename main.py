""" IMPORT STATEMENTS """
import sys
from time import time, perf_counter
import matplotlib.pyplot as plt
from SensorFusion.Vector import Vector
from SensorFusion.ComplementaryFilter import ComplementaryFilter
from SensorFusion.SensorFusion import orientation, currentPositionVelocity
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer

# MAIN PROGRAM
if __name__ == "__main__":

    # Initialize imu and magnetometer
    imu = IMU([1, 4, 7])
    mag = Magnetometer()

    # Initialize Accelerometer Vector
    vAccelerometer = Vector(*imu.get_acceleration())

    # Initialize Gyroscope Vector
    vGyroscope = Vector(*imu.get_gyroscope())
    
    # Initialize Magnetometer Vector
    vMagnetometer = Vector(*mag.get_magnetic())

    # Normalize vectors for orientation sensor fusion
    vNormAccel = vAccelerometer.normalize()
    vNormMag = vMagnetometer.normalize()

    # Obtain the initial orientation
    qAccelerometer, qMagnetometer = orientation(vNormAccel, vNormMag)
    qFirstPrediction = qAccelerometer * qMagnetometer

    # Initialize Complementary Filter with initial orientation
    compFilter = ComplementaryFilter(qFirstPrediction)

    # Initialize Position and Velocity vector
    position = Vector(0, 0, 0)
    velocity = Vector(0, 0, 0)

    out_file = open('Data/SensorFusionOutputEuler.txt', 'w')
    dataCount = 0
    num_samples = 0
    total_time = 0.0
    
    while True:

        # Obtain sensor data
        start_time = perf_counter()

        vAccelerometer.set(*imu.get_acceleration())
        vGyroscope.set(*imu.get_gyroscope())
        compFilter.currTime = time()
        vMagnetometer.set(*mag.get_magnetic())

        # Normalize vectors for orientation sensor fusion
        vNormAccel = vAccelerometer.normalize()
        vNormMag = vMagnetometer.normalize()

        # Prediction Step
        compFilter.predict(vGyroscope)

        # Correction Step
        compFilter.correctOrientation(vNormAccel, vNormMag)

        position, velocity = currentPositionVelocity(vAccelerometer, compFilter.qResult, velocity, position,
                                                     compFilter.deltaTime)
        end_time = perf_counter()
        
        num_samples += 1
        total_time += (end_time - start_time)
        
        data_rate = num_samples / total_time
        
        elev, azim, roll = compFilter.quat_to_elev_azim_roll(compFilter.qResult.normalize())
        
        plt.pause(0.0001)
        
        if dataCount < 600:
            out_file.write(str(Vector(elev, azim, roll)))
            out_file.write('\n')

        else:
            out_file.close()
            sys.exit(0)

        euler = compFilter.toEuler(compFilter.qResult)

        print(f"Data rate: {round(data_rate, 3)} Hz")
        print(f"Position: X: {position.x}, Y: {position.y}, Z: {position.z}")
        print(f"Roll: {round(euler.x, 2)}, Pitch: {round(euler.y, 2)}, Yaw: {round(euler.z, 2)}")
