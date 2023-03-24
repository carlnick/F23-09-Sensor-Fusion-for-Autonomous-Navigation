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
# import allantools

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
#calibration = dict()
#calibration['Offset_Ac_X'] = 0.0181884765625
#calibration['Offset_Ac_Y'] = -7.324218750004441e-05
#calibration['Offset_Ac_Z'] = 0.10261230468749999
#calibration['Scale_Ac_X'] = 0.9986083984375
#calibration['Scale_Ac_Y'] = 1.0138427734375
#calibration['Scale_Ac_Z'] = -1.0182373046875
#calibration['Offset_Gy_X'] = -2.4717557251908397
#calibration['Offset_Gy_Y'] = -0.333587786259542
#calibration['Offset_Gy_Z'] = -2.2610687022900766


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
    ac_x = ((((read_raw_data(ACCEL_XOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_X']))# * calibration['Scale_Ac_X'])
    ac_y = ((((read_raw_data(ACCEL_YOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_Y']))# * calibration['Scale_Ac_Y'])
    ac_z = ((((read_raw_data(ACCEL_ZOUT_H)/ACCEL_CONST)) - calibration['Offset_Ac_Z']))#S * calibration['Scale_Ac_Z']) + 1 # Uncomment to zero Z-axis
    
#    ac_x = read_raw_data(ACCEL_XOUT_H)
#    ac_y = read_raw_data(ACCEL_YOUT_H)
#    ac_z = read_raw_data(ACCEL_ZOUT_H) # Uncomment to zero Z-axis
    
    # Convert g to m/s
    ac_x = ac_x * GRAV_CONST
    ac_y = ac_y * GRAV_CONST
    ac_z = ac_z * GRAV_CONST
    
    return ac_x, ac_y, ac_z

def read_gyro():
    gy_x = (read_raw_data(GYRO_XOUT_H)/GRYO_CONST) - calibration['Offset_Gy_X']
    gy_y = (read_raw_data(GYRO_YOUT_H)/GRYO_CONST) - calibration['Offset_Gy_Y']
    gy_z = (read_raw_data(GYRO_ZOUT_H)/GRYO_CONST) - calibration['Offset_Gy_Z']
    
#    gy_x = read_raw_data(GYRO_XOUT_H)
#    gy_y = read_raw_data(GYRO_YOUT_H)
#    gy_z = read_raw_data(GYRO_ZOUT_H)
    
    return gy_x, gy_y, gy_z
    
def read_temp():
    return read_raw_data(TEMP_OUT_H)/TEMP_CONST

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

x_min = accel_sample_average(ACCEL_XOUT_H)
x_max = accel_sample_average(ACCEL_XOUT_H)
y_min = accel_sample_average(ACCEL_YOUT_H)
y_max = accel_sample_average(ACCEL_YOUT_H)
z_min = accel_sample_average(ACCEL_ZOUT_H)
z_max = accel_sample_average(ACCEL_ZOUT_H)

# calibration['Offset_Ac_X'] = (x_min + x_max) / 2
# calibration['Offset_Ac_Y'] = (y_min + y_max) / 2
# calibration['Offset_Ac_Z'] = (z_min + z_max) / 2
# calibration['Scale_Ac_X'] = x_max - calibration['Offset_Ac_X']
# calibration['Scale_Ac_Y'] = y_max - calibration['Offset_Ac_Y']
# calibration['Scale_Ac_Z'] = z_max - calibration['Offset_Ac_Z']

# print("X Offset:\t" + str(calibration['Offset_Ac_X']))
# print("Y Offset:\t" + str(calibration['Offset_Ac_Y']))
# print("Z Offset:\t" + str(calibration['Offset_Ac_Z']))
# 
# print("X Scale:\t" + str(calibration['Scale_Ac_X']))
# print("Y Scale:\t" + str(calibration['Scale_Ac_Y']))
# print("Z Scale:\t" + str(calibration['Scale_Ac_Z']))
# 
# #calibration['Offset_Gy_X'] = gyro_sample_average(GYRO_XOUT_H)
# #calibration['Offset_Gy_Y'] = gyro_sample_average(GYRO_YOUT_H)
# #calibration['Offset_Gy_Z'] = gyro_sample_average(GYRO_ZOUT_H)
# 
# print("X Offset:\t" + str(calibration['Offset_Gy_X']))
# print("Y Offset:\t" + str(calibration['Offset_Gy_Y']))
# print("Z Offset:\t" + str(calibration['Offset_Gy_Z']))

# TESTING CONSTANTS
SAMPLE_AMOUNT = 100
PRINT = True

# Sample Time (s)
SAMPLE_TIME = 0.1

# Data Array Dictionary
data = dict()
data['Accelerometer'] = dict()
data['Accelerometer']['Acceleration'] = dict()
data['Accelerometer']['Velocity'] = dict()
data['Accelerometer']['Position'] = dict()
data['Gyroscope'] = dict()
data['Gyroscope']['Angular Velocity'] = dict()
data['Gyroscope']['Angle'] = dict()

data['Time'] = []
data['Accelerometer']['Acceleration']['X'] = []
data['Accelerometer']['Acceleration']['Y'] = []
data['Accelerometer']['Acceleration']['Z'] = []
data['Accelerometer']['Velocity']['X'] = []
data['Accelerometer']['Velocity']['Y'] = []
data['Accelerometer']['Velocity']['Z'] = []
data['Accelerometer']['Position']['X'] = []
data['Accelerometer']['Position']['Y'] = []
data['Accelerometer']['Position']['Z'] = []
data['Gyroscope']['Angular Velocity']['X'] = []
data['Gyroscope']['Angular Velocity']['Y'] = []
data['Gyroscope']['Angular Velocity']['Z'] = []
data['Gyroscope']['Angle']['X'] = []
data['Gyroscope']['Angle']['Y'] = []
data['Gyroscope']['Angle']['Z'] = []
data['Temperature'] = []

def smooth_data(prev_data, data, history=50):
    return medfilt(prev_data[-history:] + [data], kernel_size=1)[-1]

# Read Data Function
def read_data():
    global prev_time
    global prev_vel_x
    global prev_vel_y
    global prev_vel_z
    global prev_pos_x
    global prev_pos_y
    global prev_pos_z
    global prev_ang_x
    global prev_ang_y
    global prev_ang_z
    
    # Read Data
    #curr_time = time()
    #curr_time = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    curr_time = datetime.utcnow().strftime('%H:%M:%S.%f')[:-3]
    accel_x, accel_y, accel_z = read_accel()
    avel_x, avel_y, avel_z = read_gyro()
    temp = read_temp()
    
    # Smooth Data
#     accel_x = smooth_data(data['Accelerometer']['Acceleration']['X'], accel_x)
#     accel_y = smooth_data(data['Accelerometer']['Acceleration']['Y'], accel_y)
#     accel_z = smooth_data(data['Accelerometer']['Acceleration']['Z'], accel_z)
#     avel_x = smooth_data(data['Gyroscope']['Angular Velocity']['X'], avel_x)
#     avel_y = smooth_data(data['Gyroscope']['Angular Velocity']['Y'], avel_y)
#     avel_z = smooth_data(data['Gyroscope']['Angular Velocity']['Z'], avel_z)
    
    # Calculate Data
    #time_delta = curr_time - prev_time
#     vel_x = calc_vel(prev_vel_x, accel_x, time_delta)
#     vel_y = calc_vel(prev_vel_y, accel_y, time_delta)
#     vel_z = calc_vel(prev_vel_z, accel_z, time_delta)
#     pos_x = calc_pos(prev_pos_x, prev_vel_x, accel_x, time_delta)
#     pos_y = calc_pos(prev_pos_y, prev_vel_y, accel_y, time_delta)
#     pos_z = calc_pos(prev_pos_z, prev_vel_z, accel_z, time_delta)
#     ang_x = calc_angle(prev_ang_x, avel_x, time_delta)
#     ang_y = calc_angle(prev_ang_y, avel_y, time_delta)
#     ang_z = calc_angle(prev_ang_z, avel_z, time_delta)
    
    # Update Previous Data
    prev_time = curr_time
#     prev_vel_x = vel_x
#     prev_vel_y = vel_y
#     prev_vel_z = vel_z
#     prev_pos_x = pos_x
#     prev_pos_y = pos_y
#     prev_pos_z = pos_z
#     prev_ang_x = ang_x
#     prev_ang_y = ang_y
#     prev_ang_z = ang_z
    
    if PRINT:
        #print(("{: ^15.5f}|" * 17).format(time_delta, accel_x, accel_y, accel_z, vel_x, vel_y, vel_z, pos_x, pos_y, pos_z, avel_x, avel_y, avel_z, ang_x, ang_y, ang_z, temp))
        print(("{: ^10}\t" + "{: ^10.5f}\t" * 7).format(curr_time, accel_x, accel_y, accel_z, avel_x, avel_y, avel_z, temp))

    data['Time'].append(curr_time)
    data['Accelerometer']['Acceleration']['X'].append(accel_x)
    data['Accelerometer']['Acceleration']['Y'].append(accel_y)
    data['Accelerometer']['Acceleration']['Z'].append(accel_z)
#     data['Accelerometer']['Velocity']['X'].append(vel_x)
#     data['Accelerometer']['Velocity']['Y'].append(vel_y)
#     data['Accelerometer']['Velocity']['Z'].append(vel_z)
#     data['Accelerometer']['Position']['X'].append(pos_x)
#     data['Accelerometer']['Position']['Y'].append(pos_y)
#     data['Accelerometer']['Position']['Z'].append(pos_z)
    data['Gyroscope']['Angular Velocity']['X'].append(avel_x)
    data['Gyroscope']['Angular Velocity']['Y'].append(avel_y)
    data['Gyroscope']['Angular Velocity']['Z'].append(avel_z)
#     data['Gyroscope']['Angle']['X'].append(ang_x)
#     data['Gyroscope']['Angle']['Y'].append(ang_y)
#     data['Gyroscope']['Angle']['Z'].append(ang_z)
    data['Temperature'].append(temp)
    
# Initialize Scheduler
s = scheduler(time, sleep)

# Setup Event
def event(countdown):
    if ((countdown > 0)):
        read_data()
        s.enter(SAMPLE_TIME, 1, event, argument=(countdown - 1,))
        s.run()
    elif (countdown < 0):
        read_data()
        s.enter(SAMPLE_TIME, 1, event, argument=(countdown,))
        s.run()

# Initialize Variables
# Run for 20 min
max_time = 7200
initial_time = time()
prev_time = initial_time
prev_vel_x = 0
prev_vel_y = 0
prev_vel_z = 0
prev_pos_x = 0
prev_pos_y = 0
prev_pos_z = 0
prev_ang_x = 0
prev_ang_y = 0
prev_ang_z = 0
        
# Start Sampling
if PRINT:
    print(("{:^15}\t" * 8).format("TIME", "X ACCEL (m/s^2)", "Y ACCEL (m/s^2)", "Z ACCEL (m/s^2)",
                                     "X ANG VEL (\u00b0/s)", "Y ANG VEL (\u00b0/s)", "Z ANG VEL (\u00b0/s)",
                                     "TEMP (\u00b0C)"))
while True:
    current_time = time()
    elapsed_time = current_time - initial_time
    
    read_data()
    s.enter(SAMPLE_TIME, 1, event, argument=(SAMPLE_AMOUNT,))
    s.run()
    
  #  if elapsed_time > max_time:
   #     break
print("COMPLETED")
