from pyrr import Vector3
import math
from random import random
from time import sleep
from pyquaternion import Quaternion

START_VALUE: float = -55555555
ZERO: float = 0.0
MAG_THRESHOLD: float = 50
MAG_SUM_THRESHOLD: float = 100
MAG_DELTA_THRESHOLD: float = 1

accel_threshold = Vector3()
gyro_threshold = Vector3()
mag_threshold = Vector3([MAG_THRESHOLD, MAG_THRESHOLD, MAG_THRESHOLD])


def check_threshold(data: float, last_data: float, threshold: float) -> float:
    if not (-threshold < data < threshold):
        print("magnetometer value exceeds threshold")
        return last_data

    if abs(data - last_data) > MAG_DELTA_THRESHOLD:
        print("magnetometer value exceeds delta threshold")
        return last_data

    return data


def threshold_mag(data: Vector3, last_data: Vector3) -> Vector3:
    mag_final: Vector3 = Vector3(last_data)

    if abs(sum(data)) > MAG_SUM_THRESHOLD:
        return mag_final

    mag_final.x = check_threshold(data.x, last_data.x, mag_threshold.x)
    mag_final.y = check_threshold(data.y, last_data.y, mag_threshold.y)
    mag_final.z = check_threshold(data.z, last_data.z, mag_threshold.z)

    return mag_final


def pitch_roll_yaw(accel_data: Vector3, mag_data: Vector3) -> Vector3:
    pitch = math.atan2(-accel_data.x, math.sqrt(accel_data.y ** 2 + accel_data.z ** 2))
    roll = math.atan2(accel_data.y, accel_data.z)

    yaw_numerator = (mag_data.z * math.sin(roll)) - (mag_data.y * math.cos(roll))
    yaw_denominator = (mag_data.x * math.cos(pitch)) + (mag_data.y * math.sin(pitch) * math.sin(roll)) + (
            mag_data.z * math.sin(pitch) * math.cos(roll))
    yaw = math.atan2(yaw_numerator, yaw_denominator)

    return Vector3([pitch, roll, yaw])


def quat_mult(p: Quaternion, q: Quaternion) -> Quaternion:
    return p * q


def orientation(accel_data: Vector3, mag_data: Vector3 = None):
    q_accelerometer = Quaternion()
    q_magnetometer = Quaternion()
    if mag_data is None:
        q_result = Quaternion()
        p_r_y = accel_data

        sin_x = math.sin(p_r_y.x / 2)
        cos_x = math.cos(p_r_y.x / 2)
        sin_y = math.sin(p_r_y.y / 2)
        cos_y = math.cos(p_r_y.y / 2)
        sin_z = math.sin(p_r_y.z / 2)
        cos_z = math.cos(p_r_y.z / 2)

        q_result[0] = cos_y * cos_x * cos_z + sin_y * sin_x * sin_z
        q_result[1] = sin_y * cos_x * cos_z - cos_y * sin_x * sin_z
        q_result[2] = cos_y * sin_x * cos_z + sin_y * cos_x * sin_z
        q_result[3] = cos_y * cos_x * sin_z - sin_y * sin_x * cos_z

        return q_result

    if accel_data.z >= 0:
        q_accelerometer[0] = (math.sqrt((accel_data.z + 1) / 2))
        q_accelerometer[1] = (-1 * accel_data.y / math.sqrt(2 * (accel_data.z + 1)))
        q_accelerometer[2] = (accel_data.x / math.sqrt(2 * (accel_data.z + 1)))
        q_accelerometer[3] = 0
    else:
        q_accelerometer[0] = -1 * accel_data.y / math.sqrt(2 * (1 - accel_data.z))
        q_accelerometer[1] = math.sqrt((1 - accel_data.z) / 2)
        q_accelerometer[2] = 0
        q_accelerometer[3] = accel_data.x / math.sqrt(2 * (1 - accel_data.z))

    rotated_mag_field = Vector3(q_accelerometer.inverse.rotate(mag_data))
    gamma = rotated_mag_field.x ** 2 + rotated_mag_field.y ** 2

    if rotated_mag_field.x >= 0:
        q_magnetometer[0] = (math.sqrt((gamma + rotated_mag_field.x * math.sqrt(gamma)) / (2 * gamma)))
        q_magnetometer[1] = 0
        q_magnetometer[2] = 0
        q_magnetometer[3] = (rotated_mag_field.y / (math.sqrt(2 * (gamma + rotated_mag_field.x * math.sqrt(gamma)))))
    else:
        q_magnetometer[0] = rotated_mag_field.y / (math.sqrt(2 * (gamma - rotated_mag_field.x * math.sqrt(gamma))))
        q_magnetometer[1] = 0
        q_magnetometer[2] = 0
        q_magnetometer[3] = math.sqrt((gamma - rotated_mag_field.x * math.sqrt(gamma)) / (2 * gamma))

    return q_accelerometer, q_magnetometer


def imu_threshold():
    pass


def current_position_velocity(acceleration: Vector3, quaternion: Quaternion, prev_vel: Vector3, prev_pos: Vector3,
                              dt: float) -> tuple[Vector3, Vector3]:
    gravity = Vector3()
    gravity.x = 2 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y)
    gravity.y = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    gravity.z = quaternion.w ** 2 - quaternion.x ** 2 - quaternion.y ** 2 + quaternion.z ** 2

    linear_accel = Vector3(acceleration - gravity)

    velocity = Vector3(prev_vel + linear_accel * dt)

    position = Vector3(prev_pos + prev_vel * dt + ((dt ** 2) / 2) * linear_accel)

    return position, velocity


if __name__ == "__main__":
    accelerometer = Vector3()
    gyroscope = Vector3()
    magnetometer = Vector3()
    quaternion = Quaternion()

    last_accel_reading = Vector3()
    last_gyro_reading = Vector3()
    last_mag_reading = Vector3()

    while True:
        magnetometer = threshold_mag(magnetometer, last_mag_reading)
        p_r_y = pitch_roll_yaw(accelerometer, magnetometer)

        quaternion = quaternion * p_r_y

        magnetometer.x = random() * 5
        magnetometer.y = random() * 5
        magnetometer.z = random() * 5
        sleep(1.0)
