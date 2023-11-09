from math import sqrt, acos, atan2, pi, fabs
from pyrr import Vector3
from time import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt
from pyquaternion import Quaternion

GRAVITY = 9.7976


class ComplementaryFilter:
    def __init__(self, starting_orientation: Quaternion):
        self.angular_rate = Vector3()
        self.q_pure_angular = Quaternion()

        self.q_estimate = starting_orientation

        self.q_orientation = Quaternion()
        self.qd_orientation = Quaternion()

        self.current_time = 0.0
        self.last_time = time()
        self.delta_time = 0.0

        self.predicted_gravity = Vector3()
        self.epsilon = 0.95
        self.alpha_accel = 1.0
        self.alpha_mag = 0.0
        self.unit_quaternion = Quaternion([1.0, 0.0, 0.0, 0.0])

        self.q_result = Quaternion()

    def predict(self, gyro_data: Vector3) -> None:
        self.current_time = time()
        self.delta_time = self.current_time - self.last_time
        self.last_time = self.current_time

        self.angular_rate = gyro_data
        self.q_pure_angular = Quaternion([0, self.angular_rate.x, self.angular_rate.y, self.angular_rate.z])

        self.qd_orientation = -0.5 * self.q_pure_angular * self.q_estimate

        print()

        self.q_orientation = self.q_estimate + (self.qd_orientation * self.delta_time)

    def compute_alpha(self, local_acceleration_vector: Quaternion) -> float:
        error_magnitude = ((local_acceleration_vector.normalised - GRAVITY) / GRAVITY)

        if error_magnitude > 0.2:
            return 0

        if error_magnitude > 0.1:
            return (-1 * error_magnitude * 10 + 2) * error_magnitude

        return error_magnitude

    # HELP
    def correct_acceleration(self, local_frame_acceleration: Vector3) -> None:

        v_predicted_gravity = Vector3(self.q_orientation.inverse.rotate(local_frame_acceleration))

        delta_acceleration = Quaternion()

        a = sqrt(2 * (v_predicted_gravity.z + 1))

        delta_acceleration[0] = sqrt((v_predicted_gravity.z + 1) / 2)
        delta_acceleration[1] = -1 * v_predicted_gravity.y / a
        delta_acceleration[2] = v_predicted_gravity.x / a
        delta_acceleration[3] = 0

        weight = self.alpha_accel

        q_identity = Quaternion()
        angle = acos(delta_acceleration.w)

        slerp_quaternion = Quaternion.slerp(q_identity, delta_acceleration, weight)
        self.q_result = self.q_orientation * slerp_quaternion

    # HELP
    def correct_magnetic_field(self, magnetometer: Vector3) -> None:
        v_world_frame_mag = Vector3(self.q_result.inverse.rotate(magnetometer))
        gamma = v_world_frame_mag.x ** 2 + v_world_frame_mag.y ** 2

        delta_mag: Quaternion = Quaternion()

        a = sqrt(gamma)

        delta_mag[0] = sqrt((gamma + v_world_frame_mag.x * a) / (2 * gamma))
        delta_mag[1] = 0
        delta_mag[2] = 0
        delta_mag[3] = v_world_frame_mag.y / sqrt(2 * (gamma + v_world_frame_mag.x * a))

        weight = self.alpha_mag

        q_identity = Quaternion()
        angle = acos(delta_mag.w)

        slerp_quaternion = Quaternion.slerp(q_identity, delta_mag, weight)
        self.q_result = self.q_result * slerp_quaternion

    def correct_orientation(self, local_frame_acceleration: Vector3, magnetometer: Vector3) -> None:
        self.correct_acceleration(local_frame_acceleration)
        self.correct_magnetic_field(magnetometer)
        self.q_estimate = self.q_result

    def to_euler(self, q: Quaternion) -> Vector3:
        angles = Vector3()

        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
        angles.x = atan2(sinr_cosp, cosr_cosp) * 180 / pi

        # pitch (y-axis rotation)
        sinp = sqrt(fabs(1 + 2 * (q[0] * q[2] - q[1] * q[3])))
        cosp = sqrt(fabs(1 - 2 * (q[0] * q[2] - q[1] * q[3])))
        angles.y = (2 * atan2(sinp, cosp) - pi / 2) * 180 / pi

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] ** 2 + q[3] ** 2)
        angles.z = atan2(siny_cosp, cosy_cosp) * 180 / pi

        if angles.z < 0:
            angles.z += 360.0

        return angles

    def graph_result(self):
        roll_pitch_yaw: Vector3 = self.to_euler(self.q_result)

        fig = plt.figure()
        ar = fig.add_subplot(3, 1, 1)
        ap = fig.add_subplot(3, 1, 2)
        ay = fig.add_subplot(3, 1, 3)
        xs = []
        ys_r = []
        ys_p = []
        ys_y = []

        def animate(i, xs, ys_r, ys_p, ys_y):
            # Add x and y to lists
            xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
            ys_r.append(roll_pitch_yaw.x)
            ys_p.append(roll_pitch_yaw.y)
            ys_y.append(roll_pitch_yaw.z)

            # Limit x and y lists to 20 items
            xs = xs[-20:]
            ys_r = ys_r[-20:]
            ys_p = ys_p[-20:]
            ys_y = ys_y[-20:]

            # Draw x and y lists
            ar.clear()
            ar.plot(xs, ys_r)
            ap.clear()
            ap.plot(xs, ys_p)
            ay.clear()
            ay.plot(xs, ys_y)

            # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.subplots_adjust(bottom=0.30)
            plt.title('Euler Angles over Time')
            plt.ylabel('Top to bottom: Roll, Pitch, Yaw')
            return

        ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, ys_y), interval=100)
        plt.show()
