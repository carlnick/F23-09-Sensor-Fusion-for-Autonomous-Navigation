import numpy as np
import matplotlib.pyplot as plt

def read_lat_long(file_name):
    lats = []
    longs = []
    with open(file_name, "r") as file:
        for line in file:
            lat, lon = map(float, line.split())
            lats.append(lat)
            longs.append(lon)
        
    return lats, longs

def rmse(list1, list2, lat_or_long):
    for i in range(1, len(list1) + 1):
        rmse = np.sqrt(np.mean((np.array(list1[:i]) - np.array(list2[:i]))**2))

        if lat_or_long == False:
            with open("lat_rmse.txt", "a") as file:
                file.write(f"{rmse}\n")
        if lat_or_long == True:
            with open("lon_rmse.txt", "a") as file:
                file.write(f"{rmse}\n")


'''
with open("lat_rmse.txt", "w") as file:
    pass
with open("lon_rmse.txt", "w") as file:
    pass
    '''

lat1, lon1 = read_lat_long("GPS_Latitude_Longitude_tmp1.txt")
lat2, lon2 = read_lat_long("Output_Latitude_Longitude_tmp1.txt")

'''
lat_or_long = False
rmse_lat = rmse(lat1, lat2, lat_or_long)
print("done lat rmse")

lat_or_long = True
rmse_lon = rmse(lon1, lon2, lat_or_long)
'''

fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

ax1.plot(lon1, lat1)
ax1.set_title("GPS Latitude and Longitude Plot")
ax1.set_xlabel("Longitude")
ax1.set_ylabel("Latitude")

ax2.plot(lon2, lat2)
ax2.set_title("Predicted Latitude and Longitude Plot")
ax2.set_xlabel("Longitude")
ax2.set_ylabel("Latitude")

plt.show()

