from SensorFusion.Vector import Vector
import math
import numpy as np

class Quaternion:

    def __init__(self, q0=1.0, q1=0.0, q2=0.0, q3=0.0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def set_q0(self, q0):
        self.q0 = q0

    def set_q1(self, q1):
        self.q1 = q1

    def set_q2(self, q2):
        self.q2 = q2

    def set_q3(self, q3):
        self.q3 = q3
    
    def set(self, q0, q1, q2, q3):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def scale(self, scalar):
        return Quaternion(self.q0 * scalar, self.q1 * scalar, self.q2 * scalar, self.q3 * scalar)

    @staticmethod
    def rotateMultiply(quat, vector):
        result = Vector()
        result.x = (vector.x * (quat.q0**2 + quat.q1**2 - quat.q2**2 - quat.q3**2) 
                    + vector.y * (2 * (quat.q1 * quat.q2 - quat.q0 * quat.q3)) 
                    + vector.z * (2 * (quat.q1 * quat.q3 + quat.q0 * quat.q2)))

        result.y = (vector.x * (2 * (quat.q1 * quat.q2 + quat.q0 * quat.q3)) 
                    + vector.y * (quat.q0**2 - quat.q1**2 + quat.q2**2 - quat.q3**2) 
                    + vector.z * (2 * (quat.q2 * quat.q3 - quat.q0 * quat.q1)))

        result.z = (vector.x * (2 * (quat.q1 * quat.q3 - quat.q0 * quat.q2)) 
                    + vector.y * (2 * (quat.q2 * quat.q3 + quat.q0 * quat.q1)) 
                    + vector.z * (quat.q0**2 - quat.q1**2 - quat.q2**2 + quat.q3**2))

        return result

    @staticmethod
    def rotateTMultiply(quat, vector):
        result = Vector()
        result.x = (vector.x * (quat.q0**2 + quat.q1**2 - quat.q2**2 - quat.q3**2) 
                    + vector.y * (2 * (quat.q1 * quat.q2 + quat.q0 * quat.q3)) 
                    + vector.z * (2 * (quat.q1 * quat.q3 - quat.q0 * quat.q2)))

        result.y = (vector.x * (2 * (quat.q1 * quat.q2 - quat.q0 * quat.q3))
                    + vector.y * (quat.q0**2 - quat.q1**2 + quat.q2**2 - quat.q3**2) 
                    + vector.z * (2 * (quat.q2 * quat.q3 + quat.q0 * quat.q1)))

        result.z = (vector.x * (2 * (quat.q1 * quat.q3 + quat.q0 * quat.q2)) 
                    + vector.y * (2 * (quat.q2 * quat.q3 - quat.q0 * quat.q1)) 
                    + vector.z * (quat.q0**2 - quat.q1**2 - quat.q2**2 + quat.q3**2))

        return result

    def conjugate(self): 
        return Quaternion(self.q0, -self.q1, -self.q2, -self.q3)

    def dot(self, other):

        result = (self.q0 * other.q0) + (self.q1 * other.q1) + (self.q2 * other.q2) + (self.q3 * other.q3)
        return result
    
    def inverse(self):
        new_q0 = self.q0 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q1 = -1 * self.q1 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q2 = -1 * self.q2 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q3 = -1 * self.q3 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)

        return Quaternion(new_q0, new_q1, new_q2, new_q3)
    
    def normalize(self):
        magnitude = math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        
        new_q0 = self.q0 / (magnitude)
        new_q1 = self.q1 / (magnitude)
        new_q2 = self.q2 / (magnitude)
        new_q3 = self.q3 / (magnitude)

        return Quaternion(new_q0, new_q1, new_q2, new_q3)
    
    def norm(self):
        return math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)

    def __add__(self, other):
        if isinstance(other, Quaternion):
            q0 = self.q0 + other.q0
            q1 = self.q1 + other.q1
            q2 = self.q2 + other.q2
            q3 = self.q3 + other.q3
            return Quaternion(q0, q1, q2, q3)
        else:
            raise TypeError("Only Quaternions for operand types are allowed.")

    def __radd__(self, other):
        if isinstance(other, Quaternion):
            q0 = self.q0 + other.q0
            q1 = self.q1 + other.q1
            q2 = self.q2 + other.q2
            q3 = self.q3 + other.q3
            return Quaternion(q0, q1, q2, q3)
        else:
            raise TypeError("Only Quaternions for operand types are allowed.")

    def __mul__(self, other):
        if isinstance(other, Quaternion):
            q0 = (self.q0 * other.q0) - (self.q1 * other.q1) - (self.q2 * other.q2) - (self.q3 * other.q3)
            q1 = (self.q0 * other.q1) + (self.q1 * other.q0) + (self.q2 * other.q3) - (self.q3 * other.q2)
            q2 = (self.q0 * other.q2) - (self.q1 * other.q3) + (self.q2 * other.q0) + (self.q3 * other.q1)
            q3 = (self.q0 * other.q3) + (self.q1 * other.q2) - (self.q2 * other.q1) + (self.q3 * other.q0)

            return Quaternion(q0, q1, q2, q3)
        
        if isinstance(other, (float, int)):
            return Quaternion(self.q0 * other, self.q1 * other, self.q2 * other, self.q3 * other)

        if isinstance(other, Vector):
            q0 = -(self.q1 * other.x) - (self.q2 * other.y) - (self.q3 * other.z)
            q1 = (self.q0 * other.x) + (self.q2 * other.z) - (self.q3 * other.y)
            q2 = (self.q0 * other.y) - (self.q1 * other.z) + (self.q3 * other.x)
            q3 = (self.q0 * other.z) + (self.q1 * other.y) - (self.q2 * other.x)

            return Quaternion(q0, q1, q2, q3)
        else:
            raise TypeError("Only Quaternion or Numeric literals for operand types are allowed.")

    __rmul__ = __mul__
    # def __rmul__(self, other):
    #     if isinstance(other, Quaternion):
    #         q0 = (self.q0 * other.q0) - (self.q1 * other.q1) - (self.q2 * other.q2) - (self.q3 * other.q3)
    #         q1 = (self.q0 * other.q1) + (self.q1 * other.q0) + (self.q2 * other.q3) - (self.q3 * other.q2)
    #         q2 = (self.q0 * other.q2) - (self.q1 * other.q3) + (self.q2 * other.q0) + (self.q3 * other.q1)
    #         q3 = (self.q0 * other.q3) + (self.q1 * other.q2) - (self.q2 * other.q1) + (self.q3 * other.q0)

    #         return Quaternion(q0, q1, q2, q3)
        
    #     if isinstance(other, (float, int)):
    #         return Quaternion(self.q0 * other, self.q1 * other, self.q2 * other, self.q3 * other)
        
    #     if isinstance(other, Vector):
    #         q0 = -(self.q1 * other.x) - (self.q2 * other.y) - (self.q3 * other.z)
    #         q1 = (self.q0 * other.x) + (self.q2 * other.z) - (self.q3 * other.y)
    #         q2 = (self.q0 * other.y) - (self.q1 * other.z) + (self.q3 * other.x)
    #         q3 = (self.q0 * other.z) + (self.q1 * other.y) - (self.q2 * other.x)

    #         return Quaternion(q0, q1, q2, q3)
    #     else:
    #         raise TypeError("Only Quaternion or Numeric literals for operand types are allowed.")

    def __str__(self):
        return "[%.8f, %.8f, %.8f, %.8f]" % (self.q0, self.q1, self.q2, self.q3)

    @staticmethod
    def from_two_vectors(v1, v2):
        # v1 is the accelerometer vector (should align with -Z global axis)
        # v2 is the magnetometer vector (should align with X global axis)
        # Global frame assumed to be: X (North), Y (East), Z (Down)
        
        # Normalize input vectors
        v1 = v1.normalize()
        v2 = v2.normalize()

        # Orthogonalize v2 with respect to v1
        v2_orth = (v2 - v1 * v1.dot(v2)).normalize()

        # v3 orthogonal to both v1 and v2_orth
        v3 = v1.cross(v2_orth).normalize()

        # Constructing rotation matrix from the unit vectors
        R = np.array([
            [v2_orth.x, v2_orth.y, v2_orth.z],
            [v3.x, v3.y, v3.z],
            [-v1.x, -v1.y, -v1.z]
        ])

        # Convert rotation matrix to quaternion
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr+1.0) * 2  # S=4*q0
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        return Quaternion(qw, qx, qy, qz)

    @staticmethod
    def from_angular_velocity(omega, dt):
        wq = Quaternion(0, omega.x * dt/2, omega.y * dt/2, omega.z * dt/2)
        return wq

    @staticmethod
    def multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 + y1*w2 + z1*x2 - x1*z2
        z = w1*z2 + z1*w2 + x1*y2 - y1*x2
        return np.array([w, x, y, z])


    def to_array(self):
        return np.array([self.q0, self.q1, self.q2, self.q3])

    # Account for Magnetic Declanation
    @staticmethod
    def quaternion_from_declanation(declination_deg):
        declination_rad = np.radians(declination_deg)
        half_angle = declination_rad/2

        w = np.cos(half_angle)
        z = np.sin(half_angle)

        return np.array([w, 0, 0, z])
    
    @staticmethod
    def rotationMatrix_from_quaternion(q):
        # Get rotation matrix from quaternion
        # Extract values form Quaternion
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3) 
        r02 = 2 * (q1 * q3 + q0 * q2)

        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)

        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        # Rotation Matrix
        rotation_matrix = np.array ([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])
        return rotation_matrix

