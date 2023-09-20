import numpy as np

def mag_voting(mag_one, mag_two, mag_three, threshold):
    # Convert readings to NumPy arrays for efficient calculations
    x_arr = np.array(mag_one[0], mag_two[0], mag_three[0])
    y_arr = np.array(mag_one[1], mag_two[1], mag_three[1])
    z_arr = np.array(mag_one[2], mag_two[2], mag_three[2])

    # Calculate the median for each axis
    median_x = np.median(x_arr)
    median_y = np.median(y_arr)
    median_z = np.median(z_arr)

    # Find the largest outlier for each axis above the threshold
    max_outlier_x = x_arr[np.argmax(np.abs(x_arr - median_x))]
    max_outlier_y = y_arr[np.argmax(np.abs(y_arr - median_y))]
    max_outlier_z = z_arr[np.argmax(np.abs(z_arr - median_z))]

    # Check if the outliers are above the threshold
    if np.abs(max_outlier_x - median_x) > threshold:
        x_arr = x_arr[x_arr != max_outlier_x]
    if np.abs(max_outlier_y - median_y) > threshold:
        y_arr = y_arr[y_arr != max_outlier_y]
    if np.abs(max_outlier_z - median_z) > threshold:
        z_arr = z_arr[z_arr != max_outlier_z]

    # Calculate the average for each axis using the filtered values
    avg_x = np.mean(x_arr)
    avg_y = np.mean(y_arr)
    avg_z = np.mean(z_arr)

    return [avg_x, avg_y, avg_z]
