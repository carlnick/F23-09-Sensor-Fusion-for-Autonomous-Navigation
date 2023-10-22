# Used to test the magnetometer subsystem. The magnetometers will collect data for duration seconds
# return the average to compare to your phone compass heading

import time
from Magnetometer import Magnetometer

def collect_data(duration, axis1, axis2):
    mag = Magnetometer()
    sum_headings = 0
    num_samples = 0
    start_time = time.time()
    
    # collect heading data for duration seconds
    while time.time() - start_time < duration:
        sum_headings += mag.get_heading(mag.get_magnetic(), axis1, axis2)
        num_samples += 1
        time.sleep(0.01)
    
    # return average
    return sum_headings / num_samples


if __name__ == "__main__":
    
    # time in seconds
    duration = 10
    X = 0
    Y = 1
    Z = 2
    averageXY = collect_data(duration, Y, X)
    print(averageXY)
    
    # averageYZ = collect_data(duration, Y, Z)
    # print(averageYZ)
    
    # averageZX = collect_data(duration, Z, X)
    # print(averageZX)