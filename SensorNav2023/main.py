# IMPORT STATEMENTS
from SensorFusion import ComplementaryFilter, Vector
from SensorFusion.SensorFusion import orientation 
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer

# MAIN PROGRAM
if __name__ == "__main__":

    # Initialize imu and magnetometer
    imu = IMU([1, 3, 4])
    mag = Magnetometer()

    # Initialize Accelerometer Vector
    vAccelerometer = Vector()
    vAccelerometer.x = imu.get_acceleration('x')
    vAccelerometer.y = imu.get_acceleration('y')
    vAccelerometer.z = imu.get_acceleration('z')

    # Initialize Gyroscope Vector
    vGyroscope = Vector()
    vGyroscope.x = imu.get_gyroscope('x')
    vGyroscope.y = imu.get_gyroscope('y')
    vGyroscope.z = imu.get_gyroscope('z')

    # Initialize Magnetometer Vector
    vMagnetometer = Vector()
    vMagnetometer.set(*mag.get_magnetic())

    # Normalize vectors for orientation sensor fusion
    vNormAccel = vAccelerometer.normalize()
    vNormMag = vMagnetometer.normalize()

    # Obtain the initial orientation
    qFirstPrediction = orientation(vNormAccel, vNormMag)

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

        # Prediction Step
        compFilter.predict(vGyroscope)

        # Correction Step

        # Obtain Corrected Orientation