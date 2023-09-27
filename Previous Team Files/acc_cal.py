import smbus
from time import sleep

import numpy as np
import matplotlib.pyplot as plt
import datetime as dt
import matplotlib.animation as animation
from scipy.integrate import cumtrapz
from itertools import count

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

TIMESTEP = 0.01
gravity = 9.80665


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
    bus.write_byte_data(Device_Address, ACCEL_CONFIG, 0x18)

    # Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    sensitivityScaleFactor = 2048.0

    # Accelerometer and Gyrometer value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    # Concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    value /= sensitivityScaleFactor
    value -= 1.3957307
    value *= 5.6
    # value *= gravity
    return value

def low_pass_filter(input_array, filter_size):
    filtered_array = []
    for i in range(len(input_array)):
        if i < filter_size:
            filtered_array.append(sum(input_array[:i+1]) / (i+1))
        else:
            filtered_array.append(sum(input_array[i-filter_size+1:i+1]) / filter_size)
    return filtered_array

plt.style.use('fivethirtyeight')
xs = []
ys = []
index = count()
bus = smbus.SMBus(1)
Device_Address = 0x68
MPU_Init()

def animate(i):

    #zAccRaw = []

    #for i in range(0, 20):
    xAccRaw = read_raw_data(ACCEL_XOUT_H)
        #sleep(TIMESTEP)

    # zAcc = low_pass_filter(zAccRaw, 5)

    #zVel = cumtrapz(zAcc, dx = TIMESTEP, initial = 0)
    #zPos = cumtrapz(zVel, dx = TIMESTEP, initial = 0)

    xs.append(next(index))
    ys.append(xAccRaw)

    # xs = xs[-20:]
    # ys = ys[-20:]

    # ax.clear()
    # ax.plot(xs, ys)

    # plt.xticks(rotation=45, ha='right')
    # plt.subplots_adjust(bottom=0.30)
    plt.cla()
    
    plt.title("Acceleration in the X-axis over time")
    plt.ylabel("Acceleration of Z-axis (g)")

    plt.plot(xs, ys)

    #time = [i * TIMESTEP for i in range(len(zPos))]
    # plt.plot(time, zAccRaw, label = 'ZAccRaw')
    # plt.plot(time, zAcc, label = 'ZAcc')
    # plt.plot(time, zVel, label = 'ZVel')
    # plt.plot(time, zPos, label = 'ZPos')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Position')
    # plt.title('Position vs. Time')
    # plt.legend()
    # plt.show()

ani = animation.FuncAnimation(plt.gcf(), animate, 1000)
plt.show()
