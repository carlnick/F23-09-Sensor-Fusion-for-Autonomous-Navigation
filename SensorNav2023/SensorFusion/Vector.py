from math import sqrt
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