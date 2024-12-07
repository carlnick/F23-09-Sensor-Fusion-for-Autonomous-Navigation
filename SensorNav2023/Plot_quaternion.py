import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from SensorFusion.Quaternion import Quaternion

def quat_to_vector(q):
    q = q / np.linalg.norm(q)

    x = 2*(q[1]*q[3] - q[0]*q[2])
    y = 2*(q[2]*q[3] - q[0]*q[1])
    z = 1 - 2*(q[1]**2 - q[2]**2)

    return np.array([x, y, z])

def load_quats(filename):
    quaternions = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip().replace('[', '').replace(']', '').replace(',', '')
            quaternions.append([float(num) for num in line.split()])

    return np.array(quaternions)

def initialize_plot(num_files):
    fig = plt.figure(figsize=(15, 5))
    axes = []
    for i in range(num_files):
        ax = fig.add_subplot(1, num_files, i + 1, projection='3d')
        ax.set_title(f"File {i + 1}")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        axes.append(ax)
    return fig, axes

file1 = 'Q_world.txt'
file2 = 'Q_body.txt'
file3 = 'Q_kalman.txt'

quat1 = load_quats(file1)
quat2 = load_quats(file2)
quat3 = load_quats(file3)

vectors1 = np.array([quat_to_vector(q) for q in quat1])
vectors2 = np.array([quat_to_vector(q) for q in quat2])
vectors3 = np.array([quat_to_vector(q) for q in quat3])

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6), subplot_kw={'projection': '3d'})

ax1.set_xlim([-1, 1])
ax1.set_ylim([-1, 1])
ax1.set_zlim([-1, 1])
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_title('Q_world')

ax2.set_xlim([-1, 1])
ax2.set_ylim([-1, 1])
ax2.set_zlim([-1, 1])
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_title('Q_body')

ax3.set_xlim([-1, 1])
ax3.set_ylim([-1, 1])
ax3.set_zlim([-1, 1])
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.set_title('Q_kalman')

quiver1, quiver2, quiver3 = None, None, None

def init():
    global quiver1, quiver2, quiver3
    quiver1 = ax1.quiver(0, 0, 0, 0, 0, 0, color = 'r', label = 'Q_world')
    quiver2 = ax2.quiver(0, 0, 0, 0, 0, 0, color = 'g', label = 'Q_body')
    quiver3 = ax3.quiver(0, 0, 0, 0, 0, 0, color = 'b', label = 'Q_kalman')
    '''
    quiver1.remove()
    quiver2.remove()
    quiver3.remove()
    '''

    return quiver1, quiver2, quiver3

def update(frame):
    global quiver1, quiver2, quiver3
    if quiver1 is not None:
        quiver1.remove()
    if quiver2 is not None:
        quiver2.remove()
    if quiver3 is not None:
        quiver3.remove()

    quiver1 = ax1.quiver(0, 0, 0, vectors1[frame, 0], vectors1[frame, 1], vectors1[frame, 2], color = 'r', label = 'Q_world')
    quiver2 = ax2.quiver(0, 0, 0, vectors2[frame, 0], vectors2[frame, 1], vectors2[frame, 2], color = 'g', label = 'Q_body')
    quiver3 = ax3.quiver(0, 0, 0, vectors3[frame, 0], vectors3[frame, 1], vectors3[frame, 2], color = 'b', label = 'Q_kalman')

    return quiver1, quiver2, quiver3

ani = animation.FuncAnimation(fig, update, frames=range(len(quat1)), init_func=init, blit=False, interval=1)

plt.show()
    










