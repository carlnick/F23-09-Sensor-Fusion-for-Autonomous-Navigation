######## Imports ########
#############################################################
import time
import random
import numpy as np
from time import perf_counter
import matplotlib.pyplot as plt
from SensorFusion.Vector import Vector
from SensorFusion.Quaternion import Quaternion
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
from SensorFusion.Position_Kalman_Filter import Position_Kalman_Filter
from GPS import GPS
from Variance_Tracker import Variance_Tracker
from SensorFusion.Madgwick_Orientation_Filter import Madgwick_Orientation_Filter
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

import json

######## Constants ########
#############################################################
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

# Initialize boolean for GPS signal
GPS_signal_found = None

if (initial_position == None):
    # No GPS signal found
    GPS_signal_found = False

    # Print a Warning indicating no GPS signal Found
    print("!WARNING - NO GPS SIGNAL FOUND. USING SIMULATED GPS VALUES!")

    # Obtain Simulated GPS Values
    simulated_lat = []
    simulated_long = []
    simulated_alt = []

    file_name = 'pos_final.json'
    with open(file_name) as data_file:
        data = json.load(data_file)

        for i in range(1, len(data)):
            currData = data[i]
            if (currData["gps_lat"] != 0):
                simulated_lat.append(currData["gps_lat"])
                simulated_long.append(currData["gps_lon"])
                simulated_alt.append(currData["gps_alt"])

    # Initialize simulated Latitude, Longitude, and Altitude Values
    initial_latitude = simulated_lat[0]
    initial_longitude = simulated_long[0]
    initial_altitude = simulated_alt[0]
else:
    # Get Latitude, Longitude, and Altitude from GPS
    initial_latitude = initial_position[0]
    initial_longitude = initial_position[1]
    initial_altitude = initial_position[2]

    # GPS signal found
    GPS_signal_found = True


# Acceleration Variances used for the Process Covariance Matrix - Accelerometer
Acc_var_x = 0.0005368133860787596
Acc_var_y = 0.0090298895217026770
Acc_var_z = 0.0001543343085982395

# Position Variance used for the Measurement Covariance Matrix - GPS
# Standard deviation for this GPS is 3.0 meters
# Found online - https://forums.adafruit.com/viewtopic.php?t=191107
gps_var = 0.000000000729

# Initialize Magnetic Declination Quaternion
q_declination = Quaternion.quaternion_from_declanation(MAGNETIC_DECLANATION_BLACKSBURG)


######## Obtain Initial Quaternion ########
#############################################################
# Initialize vector for Sensor Values
vGyroscope_array = [0, 0, 0]
vAccelerometer_array = [0, 0, 0]
vMagnetometer_array = [0, 0, 0]

# Create an empty list to hold sensor values
vGyroscope_list = []
vAccelerometer_list = []
vMagnetometer_list = []

print("Obtaining Initial Quaternion... ")

# Obtain 100 samples of initial Sensor measurements
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

# Obtain Initial Quaternion - World Frame
initial_madgwick_world = Madgwick(gyr=vGyroscope_array, acc=vAccelerometer_array, mag=vMagnetometer_array)
q_world = initial_madgwick_world.Q
q_world = q_world[-1]

# Obtain Initial Quaternion - Body Frame
initial_madgwick_body = Madgwick(gyr=vGyroscope_array, acc=vAccelerometer_array)
q_body = initial_madgwick_body.Q
q_body = q_body[-1]

print("Initial Quaternion (World Frame): ", q_world)
print("Initial Quaternion (Body Frame): ", q_body)

# Create a madgwick Orientation Filter for the world frame
madgwick_world = Madgwick_Orientation_Filter(q=q_world, gyr=vGyroscope_array, acc=vAccelerometer_array, mag=vMagnetometer_array, MARG=True)

# Create a madgwick Orientation Filter for the body frame
madgwick_body = Madgwick_Orientation_Filter(q=q_body, gyr=vGyroscope_array, acc=vAccelerometer_array, mag=vMagnetometer_array, MARG=False)


######## Main Loop ########
#############################################################
# Keep track of overall number of samples and number of GPS samples
num_samples = 0
gps_samples = 1

# Start Stopwatch to initialize time
old_t = perf_counter()

# Initialize Position Kalman filter objects - Latitude, Longitude, and Altitude
kf_east = Position_Kalman_Filter(initial_longitude, 0, gps_var, Acc_var_x, old_t)
kf_north = Position_Kalman_Filter(initial_latitude, 0, gps_var, Acc_var_y, old_t)
kf_alt = Position_Kalman_Filter(initial_altitude, 0, gps_var, Acc_var_z, old_t)

# Initialize lists for Latitude, Longitude, and Altitude
curr_pos_list_lat = []
curr_pos_list_long = []
curr_pos_list_alt = []

# Main Loop
while True:
    # Update Sensors
    vGyroscope = Vector(*imu.get_gyroscope())
    vGyroscope_array = vGyroscope.to_array()

    vAccelerometer = Vector(*imu.get_acceleration())
    vAccelerometer_array = vAccelerometer.to_array()
    vAccelerometer_array = vAccelerometer_array / GRAVITY

    vMagnetometer = Vector(*mag.get_magnetic()).normalize()
    vMagnetometer_array = vMagnetometer.to_array()

    # Obtain Quaternion from the Madgwick Orientation Filter - With respect to the World Frame
    q_world = madgwick_world.update_MARG(vGyroscope_array, vAccelerometer_array, vMagnetometer_array)

    # Obtain Quaternion from the Madgwick Orientation Filter - With respect to the body Frame
    q_body = madgwick_body.update_IMU(vGyroscope_array, vAccelerometer_array)

    # Account Quaternion for Magnetic Declination - With respect to the World Frame
    q_dec = Quaternion.multiply(q_world, q_declination)

    # Obtain Rotation Matrix from Quaternion - With respect to the World Frame
    rotation_matrix_world = Quaternion.rotationMatrix_from_quaternion(q_dec)

    # Obtain Rotation Matrix from Quaternion - With respect to the World Frame
    rotation_matrix_body = Quaternion.rotationMatrix_from_quaternion(q_body)

    # Obtain Absolute Acceleration with respect to the world frame (East, North, Up)
    absolute_acceleration = np.linalg.inv(rotation_matrix_world) @ vAccelerometer_array
    #print("Abs2", np.array2string(absolute_acceleration, formatter={'float_kind': lambda x: f"{x:.4f}"}))

    # Normalize absolute acceleration with respect to gravity
    gravity_vector = np.array([0, 0, -1])
    dynamic_gravity = np.dot(rotation_matrix_body, gravity_vector)
    #print(dynamic_gravity)
    #absolute_acceleration += dynamic_gravity
    #print("Abs", np.array2string(vAccelerometer_array, formatter={'float_kind': lambda x: f"{x:.4f}"}))
    #print("Abs", np.array2string(absolute_acceleration, formatter={'float_kind': lambda x: f"{x:.4f}"}))

    # Make a Prediction of the State based on the absolute acceleration reading
    curr_time = perf_counter() # Obtain the current time
    kf_north.predict(absolute_acceleration[0], curr_time)
    kf_east.predict(absolute_acceleration[1], curr_time)

    # A GPS reading will be sent every 500 samples of the Sensors (Accelerometer, Gyroscope, Magnetometer)
    # This is roughtly every 10 seconds
    if (num_samples % 100 == 0):
        if (GPS_signal_found): # If GPS signal update GPS and get a new positon
            last_known_gps = gps._get_position()

            # Obtain GPS Values from GPS sensor
            gps_coor_lat = last_known_gps[0]
            gps_coor_long = last_known_gps[1]

            # Convert GPS Values to Meter values
            gps_coor_lat = GPS.latToMtrs(gps_coor_lat)
            gps_coor_long = GPS.longToMtrs(gps_coor_long)

            gps_coor_alt = last_known_gps[2]

        else: # Else use simulated GPS values to update GPS data

            # Obtain GPS Values from Simualted Values
            gps_coor_lat = simulated_lat[gps_samples]
            gps_coor_long = simulated_long[gps_samples]

            # Convert GPS Values to Meter values
            gps_coor_lat = GPS.latToMtrs(gps_coor_lat)
            gps_coor_long = GPS.longToMtrs(gps_coor_long)

        # Update State based on the new GPS value
        curr_time = perf_counter()
        kf_north.update(gps_coor_lat, absolute_acceleration[0], curr_time)
        kf_east.update(gps_coor_long, absolute_acceleration[1], curr_time)

        if (gps_samples == 700):
            break

        # Increment Number of GPS Samples
        gps_samples += 1

    # Get the Current State (Position)
    curr_pos_lat = kf_north.get_position()
    curr_pos_long = kf_east.get_position()

    # Convert Meters to Latitude/Longitude Coordinates
    estimated_lat, estimated_lon = GPS.mtrsToGPS(curr_pos_lat, curr_pos_long)

    print(estimated_lat, estimated_lon)

    # Increment Overall Number of Samples
    num_samples += 1

    # Keep track of Latitude and Longitude Values
    curr_pos_list_lat.append(estimated_lat)
    curr_pos_list_long.append(estimated_lon)

print(np.var(curr_pos_list_lat))
print(np.var(curr_pos_list_long))
plt.figure(1)
plt.plot(curr_pos_list_lat, curr_pos_list_long)
plt.show()


