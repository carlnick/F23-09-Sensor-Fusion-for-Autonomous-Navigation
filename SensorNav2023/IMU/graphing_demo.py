import matplotlib.pyplot as plt
import matplotlib.animation as animation
from IMU import IMU

imu = IMU([1, 4, 7])

# Parameters
x_len = 100  # Number of points to display
y_range = [-2, 2]  # Range of possible Y values to display

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(2, 3, 1)
bx = fig.add_subplot(2, 3, 2)
cx = fig.add_subplot(2, 3, 3)
dx = fig.add_subplot(2, 3, 4)
ex = fig.add_subplot(2, 3, 5)
fx = fig.add_subplot(2, 3, 6)
xs = list(range(0, 100))
ya = [0] * x_len
yb = [0] * x_len
yc = [0] * x_len
yd = [0] * x_len
ye = [0] * x_len
yf = [0] * x_len
ax.set_ylim(y_range)
bx.set_ylim(y_range)
cx.set_ylim(y_range)
dx.set_ylim(y_range)
ex.set_ylim(y_range)
fx.set_ylim(y_range)

line, = ax.plot(xs, ya)
line2, = bx.plot(xs, yb)
line3, = cx.plot(xs, yc)
line4, = dx.plot(xs, yd)
line5, = ex.plot(xs, ye)
line6, = fx.plot(xs, yf)

ax.set_title("Acceleration in the X-axis")
bx.set_title("Acceleration in the Y-axis")
cx.set_title("Acceleration in the Z-axis")

dx.set_title("Angular Velocity in the X-axis")
ex.set_title("Angular Velocity in the Y-axis")
fx.set_title("Angular Velocity in the Z-axis")

ax.set_ylabel("Acceleration in meters per second^2")
dx.set_ylabel("Angular Velocity in degrees per second")


# This function is called periodically from FuncAnimation
def animate(i, ya, yb, yc, yd, ye, yf):
    # Add y to list
    accelerometer_data = imu.get_acceleration()
    gyroscope_data = imu.get_gyroscope()

    ya.append(accelerometer_data[0])
    yb.append(accelerometer_data[1])
    yc.append(accelerometer_data[2])
    yd.append(gyroscope_data[0])
    ye.append(gyroscope_data[1])
    yf.append(gyroscope_data[2])

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
