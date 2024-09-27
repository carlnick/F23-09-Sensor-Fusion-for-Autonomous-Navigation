import time
from time import perf_counter
from SensorFusion.Vector import Vector
from SensorFusion.Quaternion import Quaternion
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
from SensorFusion.KalmanFilter import KalmanFilter
from SensorFusion.MadgwickOrientationFilter import Madgwick_Orientation_Filter
from GPS import GPS
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common import Quaternion


# Gravity Constant for Blacksburg VA
GRAVITY = 9.7976

# Magnetic Declanation Constant for Blacksburg VA
MAGNETIC_DECLANATION_BLACKSBURG = -9.27
magnetic_declanation_matrix = np.array([[np.cos(MAGNETIC_DECLANATION_BLACKSBURG), -np.sin(MAGNETIC_DECLANATION_BLACKSBURG)], 
                                        [np.sin(MAGNETIC_DECLANATION_BLACKSBURG),  np.cos(MAGNETIC_DECLANATION_BLACKSBURG)]])


######## Initializtion ########
# Initialize IMU and Magnetometer
imu = IMU([1, 4, 7])
mag = Magnetometer()

# Initial quaternion state from sensor readings
#vAccelerometer = Vector(*imu.get_acceleration()).normalize()
#vMagnetometer = Vector(*mag.get_magnetic()).normalize()
#qInitial = Quaternion.from_two_vectors(vAccelerometer, vMagnetometer)

# Initialize GPS
gps = GPS()
initial_position = gps._get_initial_position() # Get GPS Initial Position
initial_latitude = initial_position[0]
initial_longitude = initial_position[1]
initial_altitude = initial_position[2]

# Because there are some issues with GPS signal in Whitemore, I have included the latitude, longitude, and altitude of Whitemore which will be used as the initial values.
# The real system will use measurements from the GPS
initial_latitude = 37.231010
initial_longitude = -80.42448
initial_altitude = 2097.54  #measured in feet

# Acceleration Variances used for the Process Covariance Matrix - Accelerometer
Acc_var_x = 1.88229e-5
Acc_var_y = 3.93614e-5
Acc_var_z = 1.90236e-9

# Position Variance used for the Measurement Covariance Matrix - GPS
# Standard deviation for this GPS is 3.0 meters
# Found online - https://forums.adafruit.com/viewtopic.php?t=191107
latitude_var = 3**2
longitude_var = 3**2

# Start Stopwatch to initialize time
initial_time = perf_counter()

# Initialize Kalman filter with initial quaternion
#kf_east = KalmanFilter(initial_longitude, '''initialvelocity''', Acc_var_x, longitude_var, initial_time)
#kf_north = KalmanFilter(initial_latitude, '''initialvelocity''', Acc_var_y, )
#kf_up = KalmanFilter()


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

    #vAccelerometer = Vector(*imu.get_acceleration()).normalize()
    vAccelerometer = Vector(*imu.get_acceleration())
    vAccelerometer_array = vAccelerometer.to_array()
    vAccelerometer_list.append(vAccelerometer_array)

    #vMagnetometer = Vector(*mag.get_magnetic()).normalize()
    vMagnetometer = Vector(*mag.get_magnetic())
    vMagnetometer_array = vMagnetometer.to_array()
    vMagnetometer_list.append(vMagnetometer_array)

vGyroscope_array = np.vstack(vGyroscope_list)
vAccelerometer_array = np.vstack(vAccelerometer_list)
vMagnetometer_array = np.vstack(vMagnetometer_list)

madgwick = Madgwick(gyr=vGyroscope_array, acc=vAccelerometer_array, mag=vMagnetometer_array)
q = madgwick.Q
q = q[99]

#madgwick = Madgwick()
#q = np.array([1.0, 0.0, 0.0, 0.0])

print("Initial Quaternion: ", q)
#############################################################

# Main loop
num_samples = 0
total_time = 0.0

accelerometer_values_x = []
accelerometer_values_y = []
accelerometer_values_z = []

while True:
    start_time = perf_counter()

    # Sensor updates
    vGyroscope = Vector(*imu.get_gyroscope())
    vGyroscope_array = vGyroscope.to_array()

    #vAccelerometer = Vector(*imu.get_acceleration()).normalize()
    vAccelerometer = Vector(*imu.get_acceleration())
    vAccelerometer_array = vAccelerometer.to_array()
    vAccelerometer_array = vAccelerometer_array / GRAVITY

    vMagnetometer = Vector(*mag.get_magnetic()).normalize()
    #vMagnetometer = Vector(*mag.get_magnetic())
    vMagnetometer_array = vMagnetometer.to_array()

    # Obtain Quaternion from the Madgwick Orientation Filter
    q = madgwick.updateMARG(q, vGyroscope_array, vAccelerometer_array, vMagnetometer_array)
    #print(q)

    # Get rotation matrix from quaternion
    # Extract values form Quaternion
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3) 
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # Rotation Matrix
    rotation_matrix = np.array ([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    # Obtain Absolute Acceleration with respect to the world frame (East, North, Up)
    vAccelerometer_transpose = np.array([[vAccelerometer_array[0]], [vAccelerometer_array[1]], [vAccelerometer_array[2]]])
    absolute_acceleration = np.linalg.inv(rotation_matrix) @ vAccelerometer_transpose

    # Normalize absolute acceleration with respect to gravity
    normalized_gravity = np.array([[0], [0], [1]])
    normalized_gravity = np.linalg.inv(rotation_matrix) @ normalized_gravity
    gravity_vector_x = 2*(q1*q3 - q0*q2)
    print("gravity vector x:", gravity_vector_x)
    #gravity_vector_y = 2*(q0*q1 + q2*q3)
    #gravity_vector_z = q0**2 - q1**2 - q2**2 - q3**2
    #normalized_gravity = np.array([[gravity_vector_x], [gravity_vector_y], [gravity_vector_z]])
    #print("Normalized Gravity: ", normalized_gravity)

    #absolute_acceleration -= normalized_gravity
    #print("Absolute Acceleration", absolute_acceleration)

    # Absolute Acceleration in East, North and Up
    absolute_acceleration_East = absolute_acceleration[0]
    absolute_acceleration_North = absolute_acceleration[1]
    absolute_acceleration_Up = absolute_acceleration[2]

    # Account for magnetic declanation
    absolute_acceleration_east_north = np.array([absolute_acceleration_East, absolute_acceleration_North])
    magnetic_declanation_offset = np.dot(magnetic_declanation_matrix, absolute_acceleration_east_north)

    # True Absolute Acceleration accounting for Gravity and Magnetic Declanation
    absolute_acceleration = np.array([[magnetic_declanation_offset[0]], [magnetic_declanation_offset[1]], [absolute_acceleration_Up]])
    absolute_acceleration = np.array([magnetic_declanation_offset[0], magnetic_declanation_offset[1], absolute_acceleration_Up])
    #print("Absolute Acceleration M: \n", absolute_acceleration)
    time.sleep(0.1)


    '''if (num_samples <= 5000):
        accelerometer_values_x.append(vAccelerometer_array[0])
        accelerometer_values_y.append(vAccelerometer_array[1])
        accelerometer_values_z.append(vAccelerometer_array[2])
    
    vMagnetometer = Vector(*mag.get_magnetic()).normalize()

    # Prediction step
    kf.predict(vGyroscope, time() - start_time)

    # Measurement for correction
    # Combining normalized accelerometer and magnetometer readings
    measurement = np.concatenate([vGyroscope.to_array(), vAccelerometer.to_array(), vMagnetometer.to_array()])

    # Correction step
    kf.correct(measurement)

    # Current quaternion state
    current_orientation = kf.get_state()
    
    # Display or log the orientation here
    #print(current_orientation)

    # Handling time and loop breaks for demonstration
    num_samples += 1
    total_time += (perf_counter() - start_time)
    if (num_samples >= 5000):  # Or some other condition to exit
        break

accelerometer_variance_x = np.var(accelerometer_values_x)
accelerometer_variance_y = np.var(accelerometer_values_y)
accelerometer_variance_z = np.var(accelerometer_values_z)

print("Variance X", accelerometer_variance_x)
print("Variance Y", accelerometer_variance_y)
print("Variance Z", accelerometer_variance_z)'''
