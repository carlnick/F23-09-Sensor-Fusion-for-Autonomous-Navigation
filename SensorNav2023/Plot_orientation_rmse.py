import numpy as np
import matplotlib.pyplot as plt
from SensorFusion.Quaternion import Quaternion

def load_quats(filename):
    quaternions = []
    with open(filename, 'r') as file:
        for line in file:
            line = line.strip().replace('[', '').replace(']', '').replace(',', '')
            quaternions.append([float(num) for num in line.split()])

    return np.array(quaternions)

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def orientation_error(q1, q2):
    q1_conj = quat_conjugate(q1)
    q_error = Quaternion.multiply(q1_conj, q2)
    error_angle = 2*np.arccos(np.clip(q_error[0], -1.0, 1.0))

    return np.degrees(error_angle)

Q_world = load_quats("Q_world.txt")
Q_kalman = load_quats("Q_kalman.txt")


error = [orientation_error(q_w, q_k) for q_w, q_k in zip(Q_world, Q_kalman)]
rmse = np.sqrt(np.mean(np.square(error)))

plt.figure(figsize=(10,5))
plt.plot(error, label="Orientation Error (degrees)", color="b")
plt.axhline(rmse, color="r", linestyle="--", label=f"RMSE: {rmse:.2f} degrees")
plt.xlabel("Time Step")
plt.ylabel("Orientation Error (degrees)")
plt.title("World Frame Madgwick vs Kalman Orientation Error Over Time")
plt.legend()
plt.grid(True)
plt.show()
