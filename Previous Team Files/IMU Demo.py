#!/usr/bin/env python
# coding: utf-8

# # Adjust Window Size

# In[5]:


from IPython.display import display, HTML
display(HTML("<style>.container { width:100% !important; }</style>"))


# # Initialize Sensor and Functions

# In[6]:


# Import Libraries
import smbus
from sched import scheduler
from time import time
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import math
from datetime import datetime
from scipy.signal import medfilt
from ahrs.filters import AngularRate
import vpython

# Register Values
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
TEMP_OUT_H   = 0x41

# Constant Value
GRAV_CONST   = 9.8
ACCEL_CONST  = 2048
GRYO_CONST   = 131.0
TEMP_CONST   = 340.0

# Calibration Values
calibration = dict()
calibration['Offset_Ac_X'] = -0.27578125000000003
calibration['Offset_Ac_Y'] = -0.9100830078125
calibration['Offset_Ac_Z'] = 1.4382080078125
calibration['Scale_Ac_X'] = 0.22578125000000004
calibration['Scale_Ac_Y'] = 0.21970214843750002
calibration['Scale_Ac_Z'] = -0.3733642578124998
calibration['Offset_Gy_X'] = 139.430534351145
calibration['Offset_Gy_Y'] = -26.094656488549617
calibration['Offset_Gy_Z'] = -3.388549618320611

def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    # Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    # Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    # Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)

    # Write to Accel configuration register
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 24)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    
def read_raw_data(addr):
    # Accelerometer and Gyrometer value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    # Concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value

def read_accel():
    ac_x = ((((read_raw_data(ACCEL_XOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_X']) * calibration['Scale_Ac_X'])
    ac_y = ((((read_raw_data(ACCEL_YOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_Y']) * calibration['Scale_Ac_Y'])
    ac_z = ((((read_raw_data(ACCEL_ZOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_Z']) * calibration['Scale_Ac_Z'])
    
    # Convert g to m/s
    ac_x = ac_x * GRAV_CONST
    ac_y = ac_y * GRAV_CONST
    ac_z = ac_z * GRAV_CONST
    
    return ac_x, ac_y, ac_z

def read_gyro():
    gy_x = (read_raw_data(GYRO_XOUT_H)/GRYO_CONST) - calibration['Offset_Gy_X']
    gy_y = (read_raw_data(GYRO_YOUT_H)/GRYO_CONST) - calibration['Offset_Gy_Y']
    gy_z = (read_raw_data(GYRO_ZOUT_H)/GRYO_CONST) - calibration['Offset_Gy_Z']
    
    return gy_x, gy_y, gy_z
    
def read_temp():
    return (read_raw_data(TEMP_OUT_H)/TEMP_CONST) + 36.53

def accel_sample_average(addr, sample_size = 10):    
    accel_val = []
    for i in range(sample_size):
        accel_val.append((read_raw_data(addr)/ACCEL_CONST))
        
    return np.average(accel_val)

def gyro_sample_average(addr, sample_size = 10):
    gyro_val = []
    for i in range(sample_size):
        gyro_val.append(read_raw_data(addr)/GRYO_CONST)
        
    return np.average(gyro_val)

def calc_vel(init_vel, accel, time_delta):
    return init_vel + (accel * time_delta)

def calc_pos(init_pos, init_vel, accel, time_delta):
    return init_pos + (init_vel * time_delta) + (0.5 * accel * (time_delta ** 2))

def calc_angle(init_ang, avel, time_delta):
    return init_ang + (avel * time_delta)

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
MPU_Init()


# # Acceleration Calibration
# X-axis Down

# In[ ]:


x_min = accel_sample_average(ACCEL_XOUT_H)


# X-axis Up

# In[ ]:


x_max = accel_sample_average(ACCEL_XOUT_H)


# Y-axis Down

# In[ ]:


y_min = accel_sample_average(ACCEL_YOUT_H)


# Y-axis Up

# In[ ]:


y_max = accel_sample_average(ACCEL_YOUT_H)


# Z-axis Down

# In[ ]:


z_min = accel_sample_average(ACCEL_ZOUT_H)


# Z-axis Up

# In[ ]:


z_max = accel_sample_average(ACCEL_ZOUT_H)


# Calculate Offset and Scale

# In[ ]:


calibration['Offset_Ac_X'] = (x_min + x_max) / 2
calibration['Offset_Ac_Y'] = (y_min + y_max) / 2
calibration['Offset_Ac_Z'] = (z_min + z_max) / 2
calibration['Scale_Ac_X'] = x_max - calibration['Offset_Ac_X']
calibration['Scale_Ac_Y'] = y_max - calibration['Offset_Ac_Y']
calibration['Scale_Ac_Z'] = z_max - calibration['Offset_Ac_Z']

print("X Offset:\t" + str(calibration['Offset_Ac_X']))
print("Y Offset:\t" + str(calibration['Offset_Ac_Y']))
print("Z Offset:\t" + str(calibration['Offset_Ac_Z']))

print("X Scale:\t" + str(calibration['Scale_Ac_X']))
print("Y Scale:\t" + str(calibration['Scale_Ac_Y']))
print("Z Scale:\t" + str(calibration['Scale_Ac_Z']))


# # Gyroscope Calibration

# In[ ]:


calibration['Offset_Gy_X'] = gyro_sample_average(GYRO_XOUT_H)
calibration['Offset_Gy_Y'] = gyro_sample_average(GYRO_YOUT_H)
calibration['Offset_Gy_Z'] = gyro_sample_average(GYRO_ZOUT_H)

print("X Offset:\t" + str(calibration['Offset_Gy_X']))
print("Y Offset:\t" + str(calibration['Offset_Gy_Y']))
print("Z Offset:\t" + str(calibration['Offset_Gy_Z']))


# # Data Reading

# In[ ]:


# Sample Time (s)
SAMPLE_TIME = 0.1

# Data Array Dictionary
data = dict()
data['Raw'] = dict()
data['Filtered'] = dict()
data['Raw']['Accelerometer'] = dict()
data['Raw']['Accelerometer']['Acceleration'] = dict()
data['Raw']['Gyroscope'] = dict()
data['Raw']['Gyroscope']['Angular Velocity'] = dict()
data['Filtered']['Accelerometer'] = dict()
data['Filtered']['Accelerometer']['Acceleration'] = dict()
data['Filtered']['Gyroscope'] = dict()
data['Filtered']['Gyroscope']['Angular Velocity'] = dict()

data['Time'] = []
data['Quaternion'] = []
data['Raw']['Accelerometer']['Acceleration']['X'] = []
data['Raw']['Accelerometer']['Acceleration']['Y'] = []
data['Raw']['Accelerometer']['Acceleration']['Z'] = []
data['Raw']['Gyroscope']['Angular Velocity']['X'] = []
data['Raw']['Gyroscope']['Angular Velocity']['Y'] = []
data['Raw']['Gyroscope']['Angular Velocity']['Z'] = []
data['Raw']['Temperature'] = []
data['Filtered']['Accelerometer']['Acceleration']['X'] = []
data['Filtered']['Accelerometer']['Acceleration']['Y'] = []
data['Filtered']['Accelerometer']['Acceleration']['Z'] = []
data['Filtered']['Gyroscope']['Angular Velocity']['X'] = []
data['Filtered']['Gyroscope']['Angular Velocity']['Y'] = []
data['Filtered']['Gyroscope']['Angular Velocity']['Z'] = []
data['Filtered']['Temperature'] = []

# def lowPassFilter(raw_data, filter_data):
#     a = 1
#     b = 2
#     c = 1
#     d = 155.906
#     e = -67.387
#     f = 92.519
    
#     # Not enough data to filter
#     if len(raw_data) < 3 or len(filter_data) < 2:
#         return sum(raw_data) / len(raw_data)
    
#     # Filter data
#     return ((a * raw_data[-1] + b * raw_data[-2] + c * raw_data[-3]) + d * filter_data[-1] + e * filter_data[-2]) / f

# Read Data Function
def read_data():
    global prev_time
    global angular_rate
    
    # Read Data
    curr_time = time()
    accel_x, accel_y, accel_z = read_accel()
    avel_x, avel_y, avel_z = read_gyro()
    temp = read_temp()
    
    # Update Previous Data
    prev_time = curr_time
    
    # Add data to history
    data['Time'].append(curr_time)
    data['Raw']['Accelerometer']['Acceleration']['X'].append(accel_x)
    data['Raw']['Accelerometer']['Acceleration']['Y'].append(accel_y)
    data['Raw']['Accelerometer']['Acceleration']['Z'].append(accel_z)
    data['Raw']['Gyroscope']['Angular Velocity']['X'].append(avel_x)
    data['Raw']['Gyroscope']['Angular Velocity']['Y'].append(avel_y)
    data['Raw']['Gyroscope']['Angular Velocity']['Z'].append(avel_z)
    data['Raw']['Temperature'].append(temp)
    
    # # Pass through low pass filter
    # accel_x = lowPassFilter(data['Raw']['Accelerometer']['Acceleration']['X'], data['Filtered']['Accelerometer']['Acceleration']['X'])
    # accel_y = lowPassFilter(data['Raw']['Accelerometer']['Acceleration']['Y'], data['Filtered']['Accelerometer']['Acceleration']['Y'])
    # accel_z = lowPassFilter(data['Raw']['Accelerometer']['Acceleration']['Z'], data['Filtered']['Accelerometer']['Acceleration']['Z'])
    # avel_x = lowPassFilter(data['Raw']['Gyroscope']['Angular Velocity']['X'], data['Filtered']['Gyroscope']['Angular Velocity']['X'])
    # avel_y = lowPassFilter(data['Raw']['Gyroscope']['Angular Velocity']['Y'], data['Filtered']['Gyroscope']['Angular Velocity']['Y'])
    # avel_z = lowPassFilter(data['Raw']['Gyroscope']['Angular Velocity']['Z'], data['Filtered']['Gyroscope']['Angular Velocity']['Z'])
    # temp = lowPassFilter(data['Raw']['Temperature'], data['Filtered']['Temperature'])
    
    # # Add filtered data to history
    # data['Filtered']['Accelerometer']['Acceleration']['X'].append(accel_x)
    # data['Filtered']['Accelerometer']['Acceleration']['Y'].append(accel_y)
    # data['Filtered']['Accelerometer']['Acceleration']['Z'].append(accel_z)
    # data['Filtered']['Gyroscope']['Angular Velocity']['X'].append(avel_x)
    # data['Filtered']['Gyroscope']['Angular Velocity']['Y'].append(avel_y)
    # data['Filtered']['Gyroscope']['Angular Velocity']['Z'].append(avel_z)
    # data['Filtered']['Temperature'].append(temp)
    
    # Calculate Attitude
    #data['Quaternion'].append(angular_rate.update(data['Quaternion'][-1], [data['Filtered']['Gyroscope']['Angular Velocity']['X'][-1], data['Filtered']['Gyroscope']['Angular Velocity']['Y'][-1], data['Filtered']['Gyroscope']['Angular Velocity']['Z'][-1]]))
    
# Initialize Scheduler
s = scheduler(time, sleep)

# Initialize Variables
initial_time = time()
prev_time = initial_time
data['Quaternion'].append([1.0, 0.0, 0.0, 0.0])

# Initialize Angular Rate calculator
angular_rate = AngularRate()

# Setup Event
def event():    
    read_data()

for i in range(50):
    s.enter(SAMPLE_TIME, 1, event)
    s.run()
    
print('Completed')

velocityArray = np.zeros(50)
positionArray = np.zeros(50)
for i in range(50):
    velocityArray[i] = velocityArray[i-1] + data['Raw']['Accelerometer']['Acceleration']['X'][i] * SAMPLE_TIME
    positionArray[i] = positionArray[i-1] + velocityArray[i] * SAMPLE_TIME

x = np.arange(1, 51)
# plt.title("acc")
# plt.xlabel("x")
# plt.ylabel("y")

# plt.plot(x, data['Raw']['Accelerometer']['Acceleration']['X'], color = "green")
# plt.show()

plt.title("pos")
plt.xlabel("x")
plt.ylabel("y")
plt.plot(x, positionArray, color = "blue")
plt.show()

# In[ ]:


#get_ipython().run_line_magic('matplotlib', 'inline')

# Set figure size
# plt.rcParams['figure.figsize'] = [23.8, 40]
# plt.rcParams["figure.autolayout"] = True

# # Create figure
# plt.figure()

# # Calculate time delta
# time_delta = np.subtract(data['Time'], initial_time)

# # Plot raw acceleration data
# plt.subplot(611)
# plt.title("Raw Accelerometer Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Acceleration (m/s^2)")
# plt.plot(time_delta, data['Raw']['Accelerometer']['Acceleration']['X'], 'r-', label="X-Axis")
# plt.plot(time_delta, data['Raw']['Accelerometer']['Acceleration']['Y'], 'g--', label="Y-Axis")
# plt.plot(time_delta, data['Raw']['Accelerometer']['Acceleration']['Z'], 'b:', label="Z-Axis")
# plt.legend()

# # Plot filtered acceleration data
# plt.subplot(612)
# plt.title("Filtered Accelerometer Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Acceleration (m/s^2)")
# plt.plot(time_delta, data['Filtered']['Accelerometer']['Acceleration']['X'], 'r-', label="X-Axis")
# plt.plot(time_delta, data['Filtered']['Accelerometer']['Acceleration']['Y'], 'g--', label="Y-Axis")
# plt.plot(time_delta, data['Filtered']['Accelerometer']['Acceleration']['Z'], 'b:', label="Z-Axis")
# plt.legend()

# # Plot raw gyroscope data
# plt.subplot(613)
# plt.title("Raw Gyroscope Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Angular Velocity (\u00b0/s)")
# plt.plot(time_delta, data['Raw']['Gyroscope']['Angular Velocity']['X'], 'r-', label="X-Axis")
# plt.plot(time_delta, data['Raw']['Gyroscope']['Angular Velocity']['Y'], 'g--', label="Y-Axis")
# plt.plot(time_delta, data['Raw']['Gyroscope']['Angular Velocity']['Z'], 'b:', label="Z-Axis")
# plt.legend()

# # Plot filtered gyroscope data
# plt.subplot(614)
# plt.title("Filtered Gyroscope Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Angular Velocity (\u00b0/s)")
# plt.plot(time_delta, data['Filtered']['Gyroscope']['Angular Velocity']['X'], 'r-', label="X-Axis")
# plt.plot(time_delta, data['Filtered']['Gyroscope']['Angular Velocity']['Y'], 'g--', label="Y-Axis")
# plt.plot(time_delta, data['Filtered']['Gyroscope']['Angular Velocity']['Z'], 'b:', label="Z-Axis")
# plt.legend()

# # Plot raw temperature data
# plt.subplot(615)
# plt.title("Raw Temperature Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Temperature (\u00b0C)")
# plt.plot(time_delta, data['Raw']['Temperature'], 'r', label="Temperature")
# plt.legend()

# # Plot filtered temperature data
# plt.subplot(616)
# plt.title("Filtered Temperature Data")
# plt.xlabel("Time (s)")
# plt.ylabel("Temperature (\u00b0C)")
# plt.plot(time_delta, data['Filtered']['Temperature'], 'r', label="Temperature")
# plt.legend()

# # Plot Quaternion
# plt.subplot(617)
# plt.title("Attitude")
# plt.xlabel("Time (s)")
# plt.ylabel("Attitude (\u00b0)")
# # plt.plot(time_delta, data['Quaternion'][0], 'o', label="q1")
# plt.plot(time_delta, data['Quaternion'][1], 'r', label="q2")
# plt.plot(time_delta, data['Quaternion'][2], 'g', label="q3")
# plt.plot(time_delta, data['Quaternion'][3], 'b', label="q4")
# plt.legend()


# In[ ]:


for q in data['Quaternion']:
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    yaw = math.atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
    pitch = math.asin(-2.0*(x*z - w*y));
    roll = math.atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);
    
    print(pitch, roll, yaw)


# In[ ]:




