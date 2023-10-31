import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
in_file = open('SensorFusionOutput.txt', 'r')


def animate(i):
    # read file and draw vectors using quiver
    # all start at 0,0,0
    ax.clear()
    hold = [x.strip() for x in in_file.readline().split(',')]
    vector:np.array = [hold[0][1:],hold[1],hold[2][:-1]]

    for i in range(0,3):
        vector[i] = float(vector[i])

    angle = float(in_file.readline())

    x = np.array([1.0,0.0,0.0])  # take a unit x vector
    x[0] = x[0] + math.cos(angle) * math.sqrt(2) #x and y adjustment from angle
    x[1] = x[1] + math.sin(angle) * math.sqrt(2)
    # x[0] = x[0] + math.sqrt(2) * math.cos(angle)
    dot_prod = np.dot(x,vector)
    for i in range(0,3):
        x[i] = x[i] - ((dot_prod * vector[i]) / np.linalg.norm(vector)**2)
    x = x / np.linalg.norm(x)  # normalize it
    y = np.cross(vector, x)      # cross product with k
    # x[0] = x[0] + math.cos(angle) * math.sqrt(2)
    # x[1] = x[1] + math.sin(angle) * math.sqrt(2)
    # y[0] = y[0] - math.sin(angle) * math.sqrt(2)
    # y[1] = y[1] - math.cos(angle) * math.sqrt(2)
    # print(vector)
    # print(x)
    # print(y)

    vec1 = ax.quiver(0,0,0,vector[0],vector[1],vector[2],color='blue')
    vec2 = ax.quiver(0,0,0,x[0],x[1],x[2],color='red')
    vec3 = ax.quiver(0,0,0,y[0],y[1],y[2],color='green')

    # Draw x and y lists
    ax.set_xlim([-2,2])
    ax.set_ylim([-2,2])
    ax.set_zlim([-2,2])

    # Format plot
    # plt.xticks(rotation=45, ha='right')
    # plt.subplots_adjust(bottom=0.30)
    plt.title('Axis Angle Representation over Time')
    # plt.ylabel('Top to bottom: Roll, Pitch, Yaw')
    return vec1, vec2, vec3
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate)
plt.show()