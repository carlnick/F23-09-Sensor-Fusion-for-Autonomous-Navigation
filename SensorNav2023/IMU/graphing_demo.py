import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IMU import IMU
from pyrr import Vector3

imu = IMU([1, 4, 7])

GRAVITY = 9.7976

# Parameters
x_len = 300  # Number of points to display
y_range = [-1.5, 1.5]  # Range of possible Y values to display

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 2, 1)
bx = fig.add_subplot(1, 2, 2)

xs = list(range(0, x_len))
ya = [0] * x_len
yb = [0] * x_len
yc = [0] * x_len
yd = [0] * x_len
ye = [0] * x_len
yf = [0] * x_len
ax.set_ylim(y_range)
bx.set_ylim(y_range)

line, = ax.plot(xs, ya, color='red', label='X-axis')
line2, = ax.plot(xs, yb, color='green', label='Y-axis')
line3, = ax.plot(xs, yc, color='blue', label='Z-axis')
line4, = bx.plot(xs, yd, color='red', label='X-axis')
line5, = bx.plot(xs, ye, color='green', label='Y-axis')
line6, = bx.plot(xs, yf, color='blue', label='Z-axis')

ax.legend()
bx.legend()

ax.set_title("Raw Acceleration in g")
bx.set_title("Calibrated Acceleration in g")


# This function is called periodically from FuncAnimation
def animate(i, ya, yb, yc, yd, ye, yf):
    # Add y to list

    raw_acceleration = Vector3(imu.IMUs[0].acceleration)
    calibrated_acceleration = Vector3(imu.get_acceleration())

    raw_acceleration /= GRAVITY
    calibrated_acceleration /= GRAVITY

    ya.append(raw_acceleration[0])
    yb.append(raw_acceleration[1])
    yc.append(raw_acceleration[2])

    yd.append(calibrated_acceleration[0])
    ye.append(calibrated_acceleration[1])
    yf.append(calibrated_acceleration[2])

    # Limit y list to set number of items
    ya = ya[-x_len:]
    yb = yb[-x_len:]
    yc = yc[-x_len:]
    yd = yd[-x_len:]
    ye = ye[-x_len:]
    yf = yf[-x_len:]

    # Update line with new Y values
    line.set_ydata(ya)
    line2.set_ydata(yb)
    line3.set_ydata(yc)
    line4.set_ydata(yd)
    line5.set_ydata(ye)
    line6.set_ydata(yf)

    return line, line2, line3, line4, line5, line6,


# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
                              animate,
                              fargs=(ya, yb, yc, yd, ye, yf,),
                              interval=50,
                              blit=True)
plt.show()
