""" IMPORT STATEMENTS """
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
in_file = open('Data/SensorFusionOutputEuler.txt', 'r')
yaw_values = np.linspace(0, 2*np.pi, 100)

# PLOT GENERATOR
def update_plot(frame):
    # read file and draw vectors using quiver
    # all start at 0,0,0
    ax.clear()
    hold = [x.strip() for x in in_file.readline().split(',')]
    vector:np.array = [hold[0][1:],hold[1],hold[2][:-1]]
    for i in range(0,3):
        vector[i] = float(vector[i])
    elev = vector[0]
    azim = vector[1]
    roll = vector[2]
    ax.view_init(elev, azim, roll)
    ax.quiver(0,0,0,1,0,0, color='r', label='X Vector')
    ax.quiver(0,0,0,0,1,0, color='g', label='Y Vector')
    ax.quiver(0,0,0,0,0,1, color='b', label='Z Vector')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_zlim(0, 1)
    ax.legend()
    x = np.array([0,1])

    my_ticks = ['0','1']

    ax.set_xticks(x)                                    # set x tick positions
    ax.set_xticklabels(my_ticks)                       # set the corresponding x tick labels
    ax.set_yticks(x)   # set y tick positions
    ax.set_yticklabels(my_ticks)                       # set the corresponding x tick labels
    ax.set_zticks(x)   # set y tick positions
    ax.set_zticklabels(my_ticks)                       # set the corresponding x tick labels

ani = FuncAnimation(fig, update_plot, blit=False, interval=50)
plt.show()
