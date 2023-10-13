import time
from Magnetometer.Magnetometer import Magnetometer

def collect_data(duration):
    mag = Magnetometer()
    sum_headings = 0
    num_samples = 0
    start_time = time.time()
    
    # collect heading data for duration seconds
    while time.time() - start_time < duration:
        sum_headings += mag.get_heading(mag.get_magnetic())
        num_samples += 1
        time.sleep(0.01)
    
    # return average
    return sum_headings / num_samples


if __name__ == "__main__":
    
    # time in seconds
    duration = 10
    average = collect_data(duration)
    print(average)