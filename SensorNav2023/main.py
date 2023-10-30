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
    
    # Initialize Position and Velocity vector
    position = Vector(0, 0, 0)
    velocity = Vector(0, 0, 0)

    out_file = open('SensorFusionOutput.txt', 'w')
    dataCount = 0;

    while(True):
        # Create figure for plotting
        # fig = plt.figure()
        # ar = fig.add_subplot(3, 1, 1)
        # ap = fig.add_subplot(3, 1, 2)
        # ay = fig.add_subplot(3, 1, 3)
        # xs = []
        # ys_r = []
        # ys_p = []
        # ys_y = []

        # # This function is called periodically from FuncAnimation
        # def animate(i, xs, ys_r, ys_p, ys_y):
        #     # Obtain sensor data
        #     vAccelerometer.x = imu.get_acceleration('x')
        #     vAccelerometer.y = imu.get_acceleration('y')
        #     vAccelerometer.z = imu.get_acceleration('z')

        #     vMagnetometer.set(*mag.get_magnetic())

        #     vGyroscope.x = imu.get_gyroscope('x')
        #     vGyroscope.y = imu.get_gyroscope('y')
        #     vGyroscope.z = imu.get_gyroscope('z')

        #     # Normalize vectors for orientation sensor fusion
        #     vNormAccel = vAccelerometer.normalize()
        #     vNormMag = vMagnetometer.normalize()

        #     # Prediction Step
        #     compFilter.predict(vGyroscope)

        #     # Correction Step
        #     compFilter.correctOrientation(vNormAccel, vNormMag)
        #     # Obtain Corrected Orientation
        #     print(compFilter.toEuler(compFilter.qResult))
        #     roll_pitch_yaw:Vector = compFilter.toEuler(compFilter.qResult)
        #     # print(compFilter.qResult)
        #     # Add x and y to lists
        #     xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        #     ys_r.append(roll_pitch_yaw.x)
        #     ys_p.append(roll_pitch_yaw.y)
        #     ys_y.append(roll_pitch_yaw.z)

        #     # Limit x and y lists to 20 items
        #     xs = xs[-20:]
        #     ys_r = ys_r[-20:]
        #     ys_p = ys_p[-20:]
        #     ys_y = ys_y[-20:]

        #     # Draw x and y lists
        #     ar.clear()
        #     ar.plot(xs, ys_r)
        #     ap.clear()
        #     ap.plot(xs, ys_p)
        #     ay.clear()
        #     ay.plot(xs, ys_y)

        #     # Format plot
        #     plt.xticks(rotation=45, ha='right')
        #     plt.subplots_adjust(bottom=0.30)
        #     plt.title('Euler Angles over Time')
        #     plt.ylabel('Top to bottom: Roll, Pitch, Yaw')
        #     return

        # # Set up plot to call animate() function periodically
        # ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, ys_y), interval=100)
        # plt.show()
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

        
        position, velocity = currentPositionVelocity(vAccelerometer, compFilter.qResult, velocity, position, compFilter.deltaTime)
        # print(position)

        # Obtain Corrected Orientation
        # compFilter.graphResult()
        #if dataCount < 500:
         #   axis, angle = compFilter.toAxisAngle(compFilter.qResult)
          #  out_file.write(axis.__str__())
           # out_file.write('\n')
            #out_file.write(str(angle))
            #out_file.write('\n')
            #dataCount = dataCount + 1
        #else: 
            #out_file.close()
            #sys.exit(0)
        euler = compFilter.toEuler(compFilter.qResult)
        roll = round(euler.x, 2)
        pitch = round(euler.y, 2)
        yaw = round(euler.z, 2)
        
        print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
        
        # print(compFilter.toEuler(compFilter.qResult))
        # print(compFilter.qResult)
