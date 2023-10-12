# IMPORT STATEMENTS
from SensorFusion.Vector import Vector
from SensorFusion.ComplementaryFilter import ComplementaryFilter
from SensorFusion.SensorFusion import orientation 
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer

# MAIN PROGRAM
if __name__ == "__main__":

    # Initialize imu and magnetometer
    imu = IMU([1, 4, 7])
    mag = Magnetometer()

    # Initialize Accelerometer Vector
    vAccelerometer = Vector(*imu.get_acceleration_all())

    # Initialize Gyroscope Vector
    vGyroscope = Vector(*imu.get_gyroscope_all())

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

    while(True):

        # Obtain sensor data
        vAccelerometer.x = imu.get_acceleration('x')
        vAccelerometer.y = imu.get_acceleration('y')
        vAccelerometer.z = imu.get_acceleration('z')

        vMagnetometer.set(*mag.get_magnetic())

        vGyroscope.x = imu.get_gyroscope('x')
        vGyroscope.y = imu.get_gyroscope('y')
        vGyroscope.z = imu.get_gyroscope('z')

        # Normalize vectors for orientation sensor fusion
        vNormAccel = vAccelerometer.normalize()
        vNormMag = vMagnetometer.normalize()

        # Prediction Step
        compFilter.predict(vGyroscope)

        # Correction Step
        compFilter.correctOrientation(vNormAccel, vNormMag)
        # Obtain Corrected Orientation
        # compFilter.graphResult()
        print(compFilter.qResult)