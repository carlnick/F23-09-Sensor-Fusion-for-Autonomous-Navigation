from Quaternion import Quaternion
from Vector import Vector
import time
import math


import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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
    def __init__(self, startingOrientation):

        self.angularRate = Vector()
        self.qPureAngular = Quaternion()

        self.qEstimate = startingOrientation

        self.qOrientation = Quaternion()
        self.qdOrientation = Quaternion()

        self.currTime = 0.0
        self.lastTime = time.time_ns()
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
    def predict(self, gyro_data):
        # Here we get the angular rate from the gyroscope

        # Next we set the delta time
        self.currTime = time.time_ns()
        self.deltaTime = self.currTime - self.lastTime
        self.lastTime = self.currTime

        # Set the angular rates to the given variables
        self.angularRate = gyro_data
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
    def correctAcceleration(self, body_frame_gravity:Vector, local_frame_acceleration:Quaternion):
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
        self.alpha = self.alpha_const #* self.compute_alpha(local_frame_acceleration)
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
    def correctMagneticField(self, magnetometer:Vector, local_frame_acceleration:Quaternion):
        # obtain predicted gravity
        inv_pred = self.qResult.inverse()
        # Rotation matrix for inv_pred multiplied by the body frame gravity vector measured by the accelerometer
        rotation_result:Vector = Vector() # comment above will replace Vector()
        gamma = rotation_result.x * rotation_result.x + rotation_result.y * rotation_result.y

        delta_Mag:Quaternion = Quaternion()
        # initial delta values
        delta_Mag.q0 = math.sqrt((gamma + rotation_result.x * math.sqrt(gamma)) / (2*gamma))
        delta_Mag.q1 = 0
        delta_Mag.q2 = 0
        delta_Mag.q3 = rotation_result.y / math.sqrt(2 * (gamma + rotation_result.x * math.sqrt(gamma)))
        # compute alpha weight
        self.alpha = self.alpha_const #* self.compute_alpha(local_frame_acceleration)
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
    def correctOrientation(self, body_frame_gravity:Vector, local_frame_acceleration:Quaternion, magnetometer:Vector):
        self.correctAcceleration(body_frame_gravity, local_frame_acceleration)
        self.correctMagneticField(magnetometer, local_frame_acceleration)

    def graphResult(self):
        roll_pitch_yaw:Vector = self.toEuler(self.qResult)
        # Create figure for plotting
        fig = plt.figure()
        ar = fig.add_subplot(3, 1, 1)
        ap = fig.add_subplot(3, 1, 2)
        ay = fig.add_subplot(3, 1, 3)
        xs = []
        ys_r = []
        ys_p = []
        ys_y = []

        # This function is called periodically from FuncAnimation
        def animate(i, xs, ys_r, ys_p, ys_y):

            # Add x and y to lists
            xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
            ys_r.append(roll_pitch_yaw.x)
            ys_p.append(roll_pitch_yaw.y)
            ys_y.append(roll_pitch_yaw.z)

            # Limit x and y lists to 20 items
            xs = xs[-20:]
            ys_r = ys_r[-20:]
            ys_p = ys_p[-20:]
            ys_y = ys_y[-20:]

            # Draw x and y lists
            ar.clear()
            ar.plot(xs, ys_r)
            ap.clear()
            ap.plot(xs, ys_p)
            ay.clear()
            ay.plot(xs, ys_y)

            # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.subplots_adjust(bottom=0.30)
            plt.title('Euler Angles over Time')
            plt.ylabel('Top to bottom: Roll, Pitch, Yaw')

        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys_r, ys_p, ys_y), interval=1000)
        plt.show()
        
    
    def toEuler(self, q:Quaternion):
        angles = Vector()

        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.q0 * q.q1 + q.q2 * q.q3)
        cosr_cosp = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2)
        angles.x = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (q.q0 * q.q2 - q.q1 * q.q3))
        cosp = math.sqrt(1 - 2 * (q.q0 * q.q2 - q.q1 * q.q3))
        angles.y = 2 * math.atan2(sinp, cosp) - math.M_PI / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.q0 * q.q3 + q.q1 * q.q2)
        cosy_cosp = 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3)
        angles.z = math.atan2(siny_cosp, cosy_cosp)

        return angles
