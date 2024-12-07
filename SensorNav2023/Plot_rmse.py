
import numpy as np
import matplotlib.pyplot as plt

def read_rmse(file_name):
    rmse_values = []
    with open(file_name, "r") as file:
        for line in file:
            rmse_values.append(float(line.strip()))  # Convert to float after stripping newline
    return rmse_values

# Read data from files and convert to float
lat_rmse = read_rmse("lat_rmse.txt")
lon_rmse = read_rmse("lon_rmse.txt")

# Create the plot
fig, ax1 = plt.subplots(figsize=(7, 7))

# Plot latitude RMSE on the first y-axis
ax1.plot(lat_rmse, color="blue", label="Latitude RMSE")
ax1.set_xlabel("Index")
ax1.set_ylabel("Latitude RMSE", color="blue")
ax1.tick_params(axis="y", labelcolor="blue")

# Create a second y-axis for the longitude RMSE
ax2 = ax1.twinx()
ax2.plot(lon_rmse, color="red", label="Longitude RMSE")
ax2.set_ylabel("Longitude RMSE", color="red")
ax2.tick_params(axis="y", labelcolor="red")

# Add title and show plot
plt.title("Latitude and Longitude RMSE")
fig.tight_layout()
plt.show()

