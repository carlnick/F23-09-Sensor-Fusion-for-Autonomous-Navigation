import smbus
from time import sleep

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from itertools import count

from scipy.integrate import cumtrapz

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

TIMESTEP = 0.1
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
    sensitivityScaleFactor = 131.0 / 3.0

    # Accelerometer and Gyrometer value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    # Concatenate higher and lower value
    value = ((high << 8) | low)

    # to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    value /= sensitivityScaleFactor
    return value

def calibrate(numSamples):
    print("calibrating")
    xOffset = 0
    yOffset = 0
    zOffset = 0
    xArray = list()
    yArray = list()
    zArray = list()
    for i in range(numSamples):
        # xOffset += read_raw_data(GYRO_XOUT_H)
        xArray.append(read_raw_data(GYRO_XOUT_H))
        # yOffset += read_raw_data(GYRO_YOUT_H)
        yArray.append(read_raw_data(GYRO_YOUT_H))
        # zOffset += read_raw_data(GYRO_ZOUT_H)
        zArray.append(read_raw_data(GYRO_ZOUT_H))

        sleep(TIMESTEP)
    
    xList = low_pass_filter(xArray, 20)
    yList = low_pass_filter(yArray, 20)
    zList = low_pass_filter(zArray, 20)

    for i in range(numSamples):
        xOffset += xList[i]
        yOffset += yList[i]
        zOffset += zList[i]

    xOffset /= numSamples
    yOffset /= numSamples
    zOffset /= numSamples

    print("done calibrating")
    return xOffset, yOffset, zOffset

def low_pass_filter(input_array, filter_size):
    filtered_array = []
    for i in range(len(input_array)):
        if i < filter_size:
            filtered_array.append(sum(input_array[:i+1]) / (i+1))
        else:
            filtered_array.append(sum(input_array[i-filter_size+1:i+1]) / filter_size)
    return filtered_array

bus = smbus.SMBus(1)
Device_Address = 0x68

MPU_Init()

# totalSamples = 100


# xOffset, yOffset, zOffset = calibrate(100)

# xVel = []
# yVel = []
# zVel = []

# xRaw = []
# yRaw = []
# zRaw = []

# for i in range(0,totalSamples):
#     xVel.append(read_raw_data(GYRO_XOUT_H) - xOffset)
#     yVel.append(read_raw_data(GYRO_YOUT_H) - yOffset)
#     zVel.append(read_raw_data(GYRO_ZOUT_H) - zOffset)

#     xRaw.append(read_raw_data(GYRO_XOUT_H))
#     yRaw.append(read_raw_data(GYRO_YOUT_H))
#     zRaw.append(read_raw_data(GYRO_ZOUT_H))

#     print(i)
#     sleep(TIMESTEP)

# xPos = cumtrapz(xVel, dx = TIMESTEP, initial = 0)
# yPos = cumtrapz(yVel, dx = TIMESTEP, initial = 0)
# zPos = cumtrapz(zVel, dx = TIMESTEP, initial = 0)

# print(xOffset)
# print(yOffset)
# print(zOffset)

# time = [i * TIMESTEP for i in range(len(xPos))]
# plt.plot(time, xVel, label = 'X-axis')
# plt.plot(time, yVel, label = 'Y-axis')
# plt.plot(time, zVel, label = 'Z-axis')
# plt.xlabel('Time (s)')
# plt.ylabel('Position')
# plt.title('Position vs. Time')
# plt.legend()
# plt.show()



plt.style.use('fivethirtyeight')
xVel = []
yVel = []
zVel = []
xAxis = []
index = count()


def animate(i):

    xRaw = read_raw_data(GYRO_XOUT_H)
    yRaw = read_raw_data(GYRO_YOUT_H)
    zRaw = read_raw_data(GYRO_ZOUT_H)
    xAxis.append(next(index))
    xVel.append(xRaw - 400)
    yVel.append(yRaw + 80)
    zVel.append(zRaw + 8)
    

    plt.cla()
    
    plt.title("Velocity over Time")
    plt.xlabel("Time")
    plt.ylabel("Velocity")

    plt.plot(xAxis, xVel)
    plt.plot(xAxis, yVel)
    plt.plot(xAxis, zVel)


ani = animation.FuncAnimation(plt.gcf(), animate, 1000)
plt.show()