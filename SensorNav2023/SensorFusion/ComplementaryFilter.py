from Quaternion import Quaternion
from Vector import Vector
import time
import math
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
        self.epsilon = 0.9
        self.alpha = 0.0
        self.alpha_const = 0.5
        self.unit_quat = Quaternion(1,0,0,0)

        self.qResult = Quaternion()


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
    def compute_alpha(self, local_acceleration_vector:Quaternion):
        error_magnitude = ((local_acceleration_vector.norm() - 9.81) / 9.81)
        if error_magnitude > 0.2: weight = 0
        else: 
            if error_magnitude > 0.1: weight = (-1 * error_magnitude * 10 + 2) * error_magnitude #**SHOULD PROBABLY BE CHANGED FOR OUR SYSTEM
            else: weight = 1 * error_magnitude                #**BOTH SLOPE AND CUTOFFS
                                                            #**similar system could maybe be used for gyroscope
                                                            #**what is global reference for that? equivalent of
        return weight
    """
    Accelerometer-Based Correction.

    Corrects the predicted quaternion only in the roll and pitch components.
    """
    def correctAcceleration(self, body_frame_gravity:Quaternion, local_frame_acceleration:Quaternion):
        # obtain predicted gravity
        inv_pred = self.qOrientation.inverse()
        # Rotation matrix for inv_pred multiplied by the body frame gravity vector measured by the accelerometer
        rotation_result:Vector = Vector() # comment above will replace Vector()

        delta_Accel:Quaternion = Quaternion()
        # initial delta values
        delta_Accel.q0 = math.sqrt((rotation_result.z + 1) / 2)
        delta_Accel.q1 = -1* rotation_result.y / (math.sqrt(2 * (rotation_result.z + 1)))
        delta_Accel.q2 = rotation_result.x / (math.sqrt(2 * (rotation_result.z + 1)))
        delta_Accel.q3 = 0
        # compute alpha weight
        self.alpha = self.alpha_const * self.compute_alpha(local_frame_acceleration)
        # complete LERP or SLERP
        if delta_Accel.q0 > self.epsilon:
            lerp_quat = (1- self.alpha) * self.unit_quat + self.alpha * delta_Accel
            lerp_quat = lerp_quat.normalize()
            self.qResult = self.qOrientation * lerp_quat
        else:  
            # in progress
            qIdentity = Quaternion()
            angle = math.acos(delta_Accel.dot(qIdentity))

            slerp_quat =(math.sin((1 - self.alpha) * angle) / math.sin(angle)) * qIdentity + (math.sin(self.alpha * angle) / math.sin(angle)) * delta_Accel
            self.qResult = self.qOrientation * slerp_quat

    """
    Magnetometer-Based Correction.

    Corrects the predicted quaternion in the yaw component.
    """
    def correctMagneticField(self, magnetometer:Quaternion, local_frame_acceleration:Quaternion):
        # obtain predicted gravity
        inv_pred = self.qResult.inverse()
        # Rotation matrix for inv_pred multiplied by the body frame gravity vector measured by the accelerometer
        rotation_result:Vector = Vector() # comment above will replace Vector()
        gamma = rotation_result.x * rotation_result.x + rotation_result.y * rotation_result.y

        delta_Mag:Quaternion = Quaternion()
        # initial delta values
        delta_Mag.q0 = math.sqrt((gamma + magnetometer.x * math.sqrt(gamma)) / (2*gamma))
        delta_Mag.q1 = 0
        delta_Mag.q2 = 0
        delta_Mag.q3 = magnetometer.y / math.sqrt(2 * (gamma + magnetometer.x * math.sqrt(gamma)))
        # compute alpha weight
        self.alpha = self.alpha_const * self.compute_alpha(local_frame_acceleration)
        # complete LERP or SLERP
        if delta_Mag.q0 > self.epsilon:
            lerp_quat = (1- self.alpha) * self.unit_quat + self.alpha * delta_Mag
            lerp_quat = lerp_quat.normalize()
            self.qResult = self.qResult * lerp_quat
        else:  
            # in progress
            qIdentity = Quaternion()
            angle = math.acos(delta_Mag.dot(qIdentity))

            slerp_quat =(math.sin((1 - self.alpha) * angle) / math.sin(angle)) * qIdentity + (math.sin(self.alpha * angle) / math.sin(angle)) * delta_Mag
            self.qResult = self.qResult * slerp_quat

    """
    Correction step.

    Because each delta quaternion is computed and filtered independently,
    they are separated into two steps.
    First should be accelerometer-based correction, then magnetometer-based correction.
    """
    def correctOrientation(self):
        self.correctAcceleration()
        self.correctMagneticField()
