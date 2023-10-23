import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
in_file = open('SensorFusionOutput.txt', 'r')


def animate(i):
    # read file and draw vectors using quiver
    # all start at 0,0,0
    ax.clear()
    hold = [x.strip() for x in in_file.readline().split(',')]
    vector:np.array = [hold[0][1:],hold[1],hold[2][:-1]]
    x = np.random.randn(3)  # take a random vector
    x -= x.dot(vector) * vector / np.linalg.norm(vector)**2
    x /= np.linalg.norm(x)  # normalize it
    y = np.cross(vector, x)      # cross product with k

    ax.quiver(0,0,0,vector[0],vector[1],vector[2])
    ax.quiver(0,0,0,x[0],x[1],x[2])
    ax.quiver(0,0,0,y[0],y[1],y[2])
    print(vector[0]+ ',' + vector[1] + ',' + vector[2])

    # Draw x and y lists
    ax.set_xlim([-2,2])
    ax.set_ylim([-2,2])
    ax.set_zlim([-2,2])

    # Format plot
    # plt.xticks(rotation=45, ha='right')
    # plt.subplots_adjust(bottom=0.30)
    plt.title('Axis Angle Representation over Time')
    # plt.ylabel('Top to bottom: Roll, Pitch, Yaw')
    return
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, blit=True)
plt.show()