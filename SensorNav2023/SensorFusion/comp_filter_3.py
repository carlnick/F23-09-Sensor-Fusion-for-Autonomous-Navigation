from pyquaternion import Quaternion
from pyrr import vector, Vector3
from math import sqrt
from time import time

GRAVITY = 9.7976
BEST_STATIC_GAIN = 0.5


def get_initial_state(acceleration: Vector3, magnetic_field: Vector3) -> Quaternion:
    intial_accel = delta_acceleration_calculation(acceleration)
    initial_mag = delta_magnetic_field_calculation(magnetic_field)
    initial_state = intial_accel * initial_mag
    return initial_state


def prediction_step(angular_velocity: Vector3, previous_state: Quaternion, timestep: float) -> Quaternion:
    angular_velocity_q = Quaternion([0, angular_velocity.x, angular_velocity.y, angular_velocity.z])
    derivative = -0.5 * angular_velocity_q * previous_state
    integral = previous_state + derivative * timestep
    return integral


def correction_step(prediction: Quaternion, accel_prediction: Quaternion, mag_prediction: Quaternion) -> Quaternion:
    corrected_prediction = prediction * accel_prediction * mag_prediction
    return corrected_prediction


def delta_acceleration_calculation(acceleration: Vector3):
    delta_acceleration = Quaternion()

    if acceleration.z >= 0:
        delta_acceleration[0] = sqrt((acceleration.z + 1.0) / 2.0)
        delta_acceleration[1] = -1.0 * (acceleration.y / sqrt(2.0 * (acceleration.z + 1.0)))
        delta_acceleration[2] = (acceleration.x / sqrt(2.0 * (acceleration.z + 1.0)))
        delta_acceleration[3] = 0.0
    else:
        delta_acceleration[0] = -1.0 * (acceleration.y / sqrt(2.0 * (1.0 - acceleration.z)))
        delta_acceleration[1] = sqrt((1.0 - acceleration.z) / 2.0)
        delta_acceleration[2] = 0.0
        delta_acceleration[3] = acceleration.x / sqrt(2.0 * (1.0 - acceleration.z))

    return delta_acceleration


def accel_correction_step(prediction: Quaternion, acceleration: Vector3, gain: float) -> Quaternion:
    predicted_gravity = prediction.inverse.rotate(acceleration)
    delta_acceleration = delta_acceleration_calculation(acceleration)
    delta_acceleration_slerp = Quaternion.slerp(Quaternion(), delta_acceleration, gain)
    accel_correction = prediction * delta_acceleration_slerp
    return accel_correction


def delta_magnetic_field_calculation(world_frame_magnetic_field: Vector3):
    delta_magnetic_field = Quaternion()
    gamma = world_frame_magnetic_field.x ** 2 + world_frame_magnetic_field.y ** 2

    if world_frame_magnetic_field.x >= 0:
        delta_magnetic_field[0] = sqrt(gamma + world_frame_magnetic_field.x * sqrt(gamma)) / sqrt(2.0 * gamma)
        delta_magnetic_field[1] = 0.0
        delta_magnetic_field[2] = 0.0
        delta_magnetic_field[3] = world_frame_magnetic_field.y / sqrt(
            2.0 * (gamma + world_frame_magnetic_field.x * sqrt(gamma)))
    else:
        delta_magnetic_field[0] = world_frame_magnetic_field.y / (
                sqrt(2.0) * sqrt(gamma - world_frame_magnetic_field.x * sqrt(gamma)))
        delta_magnetic_field[1] = 0.0
        delta_magnetic_field[2] = 0.0
        delta_magnetic_field[3] = sqrt(gamma - world_frame_magnetic_field.x * sqrt(gamma)) / sqrt(2.0 * gamma)
    return delta_magnetic_field


def mag_correction_step(prediction: Quaternion, accel_correction: Quaternion, magnetic_field: Vector3,
                        gain: float) -> Quaternion:
    world_frame_magnetic_field = Vector3(accel_correction.inverse.rotate(magnetic_field))
    delta_magnetic_field = delta_magnetic_field_calculation(world_frame_magnetic_field)
    delta_magnetic_field_slerp = Quaternion.slerp(Quaternion(), delta_magnetic_field, gain)
    mag_correction = prediction * delta_magnetic_field_slerp
    return mag_correction


def gain_factor(magnitude_error: float) -> float:
    if magnitude_error <= 0.1:
        return 1.0
    if magnitude_error >= 0.2:
        return 0.0
    return -10.0 * magnitude_error + 2.0


def adaptive_gain(acceleration: Vector3):
    magnitude_error = (vector.length(acceleration) - GRAVITY) / GRAVITY
    gain = BEST_STATIC_GAIN * gain_factor(magnitude_error)
    return gain


class ComplementaryFilter:
    def __init__(self, acceleration: Vector3, magnetic_field: Vector3):
        self.estimation = get_initial_state(acceleration, magnetic_field)
        self._previous_time = time()

    def _get_timestep(self) -> float:
        timestep = time() - self._previous_time
        self.previous_time = time()
        return timestep

    def iterate(self, acceleration: Vector3, angular_velocity: Vector3, magnetic_field: Vector3) -> Quaternion:
        timestep = self._get_timestep()
        prediction = prediction_step(angular_velocity, self.estimation, timestep)
        gain = adaptive_gain(acceleration)
        accel_correction = accel_correction_step(prediction, acceleration, gain)
        mag_correction = mag_correction_step(prediction, accel_correction, magnetic_field, gain)
        self.estimation = correction_step(prediction, accel_correction, mag_correction)
        return self.estimation
