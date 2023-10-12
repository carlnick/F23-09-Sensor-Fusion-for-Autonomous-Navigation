from SensorFusion.Vector import Vector
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

    @staticmethod
    def rotateMultipy(quat, vector):
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

        new_q0 = self.q0 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q1 = self.q1 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q2 = self.q2 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        new_q3 = self.q3 / (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)

        return Quaternion(new_q0, new_q1, new_q2, new_q3)
    
    def norm(self):
        return (self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)

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

    def __rmul__(self, other):
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

    def __str__(self):
        return "[%.8f, %.8f, %.8f, %.8f]" % (self.q0, self.q1, self.q2, self.q3)