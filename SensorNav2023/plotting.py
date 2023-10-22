import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
in_file = open('SensorFusionOutput.txt', 'r')


def animate(i):
    # Add x and y to lists
    ax.clear()
    vector = [x.strip() for x in in_file.readline().split(',')]
    ax.quiver(0,0,0,vector[0][1:],vector[1],vector[2][:-1])

    print(vector[0][1:]+ ',' + vector[1] + ',' + vector[2][:-1])

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
ani = animation.FuncAnimation(fig, animate)
plt.show()