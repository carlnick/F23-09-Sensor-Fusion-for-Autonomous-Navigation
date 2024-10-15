import time
import random
from time import perf_counter
from SensorFusion.Vector import Vector
from SensorFusion.Quaternion import Quaternion
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
from SensorFusion.Position_Kalman_Filter import Position_Kalman_Filter
from SensorFusion.MadgwickOrientationFilter import Madgwick_Orientation_Filter
from GPS import GPS
import numpy as np
from ahrs.filters import Madgwick

from SensorFusion.Test_Kalman import Test_Kalman_Filter


# Gravity Constant for Blacksburg VA
GRAVITY = 9.7976

# Magnetic Declanation Constant for Blacksburg VA
MAGNETIC_DECLANATION_BLACKSBURG = -9.27


######## Initializtion ########
#############################################################
# Initialize IMU and Magnetometer
imu = IMU([1, 4, 7])
mag = Magnetometer()


# Initialize GPS
gps = GPS()
initial_position = gps._get_position() # Get GPS Initial Position
if (initial_position == None):
    # Because there are some issues with GPS signal in Whitemore, I have included the latitude, longitude, and altitude of Whitemore which will be used as the initial values.
    # The real system will use measurements from the GPS
    initial_latitude = 37.231010
    initial_longitude = -80.42448
    initial_altitude = 2097.54  #measured in feet
else:
    initial_latitude = initial_position[0]
    initial_longitude = initial_position[1]
    initial_altitude = initial_position[2]

# List of Simulated GPS Values
gps_coordinates_sim = [(37.7749 + i * 0.0001, -122.4194 + i * 0.0001) for i in range(100)]
initial_longitude = gps_coordinates_sim[0][0]

hiker_velocity = [round(random.uniform(1.0, 2.5), 2) for _ in range(100)]


# Acceleration Variances used for the Process Covariance Matrix - Accelerometer
Acc_var_x = 0.0005368133860787596
Acc_var_y = 0.0090298895217026770
Acc_var_z = 0.0001543343085982395

# Position Variance used for the Measurement Covariance Matrix - GPS
# Standard deviation for this GPS is 3.0 meters
# Found online - https://forums.adafruit.com/viewtopic.php?t=191107
gps_var = 3.0**2

# Initialize Magnetic Declination Quaternion
q_declination = Quaternion.quaternion_from_declanation(MAGNETIC_DECLANATION_BLACKSBURG)

# Initialize Position Kalman filter objects
kf_east = Position_Kalman_Filter(initial_longitude, Acc_var_x, gps_var)
kf_north = Position_Kalman_Filter(initial_latitude, Acc_var_y, gps_var)
kf_up = Position_Kalman_Filter(initial_altitude, Acc_var_z, gps_var)



######## Obtain Initial Quaternion ########
#############################################################
vGyroscope_array = [0, 0, 0]
vAccelerometer_array = [0, 0, 0]
vMagnetometer_array = [0, 0, 0]

vGyroscope_list = []
vAccelerometer_list = []
vMagnetometer_list = []

print("Obtaining Initial Quaternion... ")

# Obtain Initial Quaternion
for i in range(100):
    vGyroscope = Vector(*imu.get_gyroscope())
    vGyroscope_array = vGyroscope.to_array()
    vGyroscope_list.append(vGyroscope_array)

    vAccelerometer = Vector(*imu.get_acceleration())
    vAccelerometer_array = vAccelerometer.to_array()
    vAccelerometer_list.append(vAccelerometer_array)

    vMagnetometer = Vector(*mag.get_magnetic())
    vMagnetometer_array = vMagnetometer.to_array()
    vMagnetometer_list.append(vMagnetometer_array)

vGyroscope_array = np.vstack(vGyroscope_list)
vAccelerometer_array = np.vstack(vAccelerometer_list)
vMagnetometer_array = np.vstack(vMagnetometer_list)

madgwick = Madgwick(gyr=vGyroscope_array, acc=vAccelerometer_array, mag=vMagnetometer_array)
q = madgwick.Q
q = q[99]

print("Initial Quaternion: ", q)


######## Main Loop ########
#############################################################
num_samples = 0
gps_samples = 1

abs_accelerometer_values_x = []
abs_accelerometer_values_y = []
abs_accelerometer_values_z = []

# Start Stopwatch to initialize time
old_t = perf_counter()

test_east = Test_Kalman_Filter(initial_longitude, hiker_velocity[0], gps_var, Acc_var_x, old_t)

while True:
    # Sensor updates
    vGyroscope = Vector(*imu.get_gyroscope())
    vGyroscope_array = vGyroscope.to_array()

    vAccelerometer = Vector(*imu.get_acceleration())
    vAccelerometer_array = vAccelerometer.to_array()
    vAccelerometer_array = vAccelerometer_array / GRAVITY

    vMagnetometer = Vector(*mag.get_magnetic()).normalize()
    vMagnetometer_array = vMagnetometer.to_array()

    # Obtain Quaternion from the Madgwick Orientation Filter
    q = madgwick.updateMARG(q, vGyroscope_array, vAccelerometer_array, vMagnetometer_array)

    # Account for Magnetic Declination
    q_dec = Quaternion.multiply(q, q_declination)

    # Rotation Matrix
    rotation_matrix = Quaternion.rotationMatrix_from_quaternion(q_dec)

    # Obtain Absolute Acceleration with respect to the world frame (East, North, Up)
    absolute_acceleration = np.linalg.inv(rotation_matrix) @ vAccelerometer_array
    #print("Abs", np.array2string(absolute_acceleration, formatter={'float_kind': lambda x: f"{x:.4f}"}))

    '''
    # Normalize absolute acceleration with respect to gravity
    normalized_gravity = np.array([0, 0, 0, 1])
    normalized_gravity = np.linalg.inv(rotation_matrix) @ normalized_gravity
    '''

    curr_time = perf_counter()
    test_east.predict(absolute_acceleration[0], curr_time)

    # A boolean indicating whether GPS has been used 
    GPS_used = False

    # Obtain a change in postion relative to last know position (From output of Kalman)
    if (num_samples % 30 == 0): # A GPS coordinate will be sent in every 3 seconds (if a frequency of 0.1 seconds per loop is used)
        '''
        last_known_gps = gps._get_position()
        last_known_latitude = last_known_gps[0]
        last_known_longitude = last_known_gps[1]
        last_known_altitude = last_known_gps[2]
        '''
        gps_coor = gps_coordinates_sim[gps_samples][0]
        test_east.update(gps_coor, hiker_velocity[gps_samples])
        gps_samples += 1

        '''
        # Change in position calculation
        #delta_position_north, delta_position_east, delta_position_up, overall_distance_traveled, azimuth = GPS.change_in_position_between_two_points(last_known_gps, output_from_kalman)

        GPS_used = True

    else: # If a GPS coordinate is not used to determine a change in postion, the acceleromter will be used
        # Obtain change in time
        new_t = perf_counter()
        delta_t = new_t - old_t

        # Kinematic Calculations for Position using Absolute Acceleration - calculates change in position
        delta_position_north = absolute_acceleration[0]*delta_t**2 + 0.5*absolute_acceleration[0]*delta_t**2
        delta_position_east = absolute_acceleration[1]*delta_t**2 + 0.5*absolute_acceleration[1]*delta_t**2
        delta_position_up = absolute_acceleration[2]*delta_t**2 + 0.5*absolute_acceleration[2]*delta_t**2

    old_t = perf_counter()
    '''

    curr_pos = test_east.get_position()
    print(curr_pos)

    


    time.sleep(0.1)

    '''
    if (num_samples <= 5000):
        abs_accelerometer_values_x.append(absolute_acceleration[0])
        abs_accelerometer_values_y.append(absolute_acceleration[1])
        abs_accelerometer_values_z.append(absolute_acceleration[2])
    else:
        break
    '''
    num_samples += 1




accelerometer_variance_x = np.var(abs_accelerometer_values_x)
accelerometer_variance_y = np.var(abs_accelerometer_values_y)
accelerometer_variance_z = np.var(abs_accelerometer_values_z)

print("Variance X", accelerometer_variance_x)
print("Variance Y", accelerometer_variance_y)
print("Variance Z", accelerometer_variance_z)









