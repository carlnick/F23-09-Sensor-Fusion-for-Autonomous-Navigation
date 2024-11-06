
import numpy as np
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

class Orientation_Kalman_Filter:
    def __init__(self, initial_quaternion):
