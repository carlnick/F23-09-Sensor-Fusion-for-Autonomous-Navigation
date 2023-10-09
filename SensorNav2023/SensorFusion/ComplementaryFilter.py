from Quaternion import Quaternion
from Vector import Vector
import time
"""
Complementary Filter Class.
Creates an instance of the filter.
! Assumes all given acceleration values and magnetic fields are normalized.
"""
class ComplementaryFilter:

    """
    @constructor

    Should initialize each and every variable that will be used for filtering.
    """
    def __init__(self):

        self.angularRate = Vector()
        self.qPureAngular = Quaternion()

        self.qEstimate = Quaternion()

        self.qOrientation = Quaternion()
        self.qdOrientation = Quaternion()

        self.currTime = time.time_ns()
        self.lastTime = 0.0
        self.deltaTime = 0.0

        self.predictedGravity = Vector()

    """
    Prediction step.
    
    The initial orientation quaternion is calculated before the initialization of our filter. (Section 4)
    """
    def predict(self):
        # Here we get the angular rate from the gyroscope

        # Next we set the delta time
        self.currTime = time.time_ns()
        self.deltaTime = self.currTime - self.lastTime
        self.lastTime = self.currTime

        # Set the angular rates to the given variables
        self.angularRate.set("""new angular rates""")
        self.qPureAngular.set(0, self.angularRate.x, self.angularRate.y, self.angularRate.z)

        # Calculate this time instant's derivative of the orientation
        self.qdOrientation = -1/2 * self.qPureAngular * self.qEstimate

        # Integrate the derivative of the orientation
        self.qOrientation = self.qEstimate + (self.qdOrientation * self.deltaTime)

    #! A Quick Note about correction:
    """
    There are three main steps to correct predicted quaternion.

    1. Rotation:
        We use the inverse predicted quaternion to rotate the normalized body from gravity vector into the global frame.
        We obtain the predicted gravity vector and use it with the gravity vector to obtain our delta quaternion.

    2. Linear intERPolation (LERP) and Spherical Linear intERPolation (SLERP) piecewise.
        *Used because the frame doesn't have a large difference from the original frame.
        First, we compute the angle between q1, the identity quaternion, and the delta quaternion.
        Then, using a threshold value, we use a piecewise function.
        Above this threshold we use LERP with a gain to obtain the delta quaternion which is normalized after.
        Below this threshold we use SLERP with the gain to obtain the normalized delta quaternion.

    3. Adaptive Gain
        *Used because in highly dynamic motions, a constant gain cannot show an accurate orientation estimation.
        We define a magnitude error as a function of the norm of the measured local frame acceleration vector
        before normaliztion and gravity, 9.81 m/s^2.
        We then use the best filtering gain used with LERP and SLERP and a piecewise continuous function of
        the magnitude error called "gain factor" to obtain this instance's filtering gain.
    """

    """
    Accelerometer-Based Correction.

    Corrects the predicted quaternion only in the roll and pitch components.
    """
    def correctAcceleration(self):
        # obtain current acceleration
        acceleration = Vector()
        # obtain predicted gravity
        predictedGravity = self.qOrientation.conjugate() * acceleration * self.qOrientation
        #! This would return a Quaternion instead of a Vector so it may be better to use matrix notation.
        return

    """
    Magnetometer-Based Correction.

    Corrects the predicted quaternion in the yaw component.
    """
    def correctMagneticField(self):
        return

    """
    Correction step.

    Because each delta quaternion is computed and filtered independently,
    they are separated into two steps.
    First should be accelerometer-based correction, then magnetometer-based correction.
    """
    def correctOrientation(self):
        self.correctAcceleration()
        self.correctMagneticField()
