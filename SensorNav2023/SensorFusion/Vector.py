from math import sqrt
import numpy as np

class Vector:
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self): 
        return "[%.8f, %.8f, %.8f]" % (self.x, self.y, self.z)
    
    def sum(self):
        return (self.x + self.y + self.z)

    def set(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def normalize(self):
        magnitude = sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z))
        newX = self.x / magnitude
        newY = self.y / magnitude
        newZ = self.z / magnitude
        return Vector(newX, newY, newZ)

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other):
        return Vector(
                self.y * other.z - self.z * other.y,
                self.z * other.x - self.x * other.z,
                self.x * other.y - self.y * other.x
        )

    def normalize(self):
        norm = (self.x**2 + self.y**2 + self.z**2)**0.5
        return Vector(self.x / norm, self.y / norm, self.z / norm)

    def __mul__(self, scalar):
        if isinstance(scalar, Vector):
            return Vector(self.x * scalar.x, self.y * scalar.y, self.z * scalar.z)
        elif isinstance(scalar, (int, float)):
            return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y, self.z - other.z)

    def to_array(self):
        return np.array([self.x, self.y, self.z])
