from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector
from IMU.IMU import IMU
from Magnetometer.Magnetometer import Magnetometer
import time
import math

import numpy as np
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
        self.lastTime = time.time()
        self.deltaTime = 0.0

        self.predictedGravity = Vector()
        self.epsilon = 0.95
        self.alpha_const = 0.5
        self.unit_quat = Quaternion(1,0,0,0)

        self.qResult = Quaternion()


    """
    Prediction step.
    
    The initial orientation quaternion is calculated before the initialization of our filter. (Section 4)
    """
    def predict(self, gyro_data):
        # Here we get the angular rate from the gyroscope
        # print(self.qEstimate)
        # Next we set the delta time
        # self.currTime = time.time()
        self.deltaTime = self.currTime - self.lastTime
        self.lastTime = self.currTime

        # Set the angular rates to the given variables
        self.angularRate = gyro_data
        self.qPureAngular.set(0, self.angularRate.x, self.angularRate.y, self.angularRate.z)

        # Calculate this time instant's derivative of the orientation
        self.qdOrientation = -1/2 * self.qPureAngular * self.qEstimate
        # print(self.qPureAngular)
        # print(self.qEstimate)
        print()

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
    def correctAcceleration(self, local_frame_acceleration:Vector):
        # print(self.qOrientation)
        # Rotation matrix for inv_pred multiplied by the body frame gravity vector measured by the accelerometer
        vPredictedGravity = Quaternion.rotateTMultiply(self.qOrientation, local_frame_acceleration)
        
        # print(vPredictedGravity)

        delta_Accel:Quaternion = Quaternion()
        # initial delta values
       
        delta_Accel.q0 = math.sqrt((vPredictedGravity.z + 1) / 2)
        delta_Accel.q1 = -1* vPredictedGravity.y / (math.sqrt(2 * (vPredictedGravity.z + 1)))
        delta_Accel.q2 = vPredictedGravity.x / (math.sqrt(2 * (vPredictedGravity.z + 1)))
        delta_Accel.q3 = 0
        # compute alpha weight
        weight = self.alpha_const #* self.compute_alpha(local_frame_acceleration)
        # complete LERP or SLERP
        if delta_Accel.q0 > self.epsilon:
            lerp_quat = (1- weight) * self.unit_quat + weight * delta_Accel
            lerp_quat = lerp_quat.normalize()
            print("lerp")
            print(lerp_quat)
            print("Result before accel lerp")
            print(self.qResult)
            self.qResult = self.qOrientation * lerp_quat
        else:  
            # in progress
            qIdentity = Quaternion()
            angle = math.acos(delta_Accel.q0)

            slerp_quat =(math.sin((1 - weight) * angle) / math.sin(angle)) * qIdentity + (math.sin(weight * angle) / math.sin(angle)) * delta_Accel
            print("slerp")
            print(slerp_quat)
            print("Result before accel slerp")
            print(self.qResult)
            self.qResult = self.qOrientation * slerp_quat
        print("Result after accel Lerp/Slerp")
        print(self.qResult)

    """
    Magnetometer-Based Correction.

    Corrects the predicted quaternion in the yaw component.
    """
    def correctMagneticField(self, magnetometer:Vector):

        # Rotation matrix for inv_pred multiplied by the body frame gravity vector measured by the accelerometer
        vWorldFrameMag = Quaternion.rotateTMultiply(self.qResult, magnetometer)
        gamma = vWorldFrameMag.x**2 + vWorldFrameMag.y**2

        delta_Mag:Quaternion = Quaternion()
        # initial delta values
        delta_Mag.q0 = math.sqrt((gamma + vWorldFrameMag.x * math.sqrt(gamma)) / (2*gamma))
        delta_Mag.q1 = 0
        delta_Mag.q2 = 0
        delta_Mag.q3 = vWorldFrameMag.y / math.sqrt(2 * (gamma + vWorldFrameMag.x * math.sqrt(gamma)))
        # compute alpha weight
        weight = self.alpha_const #* self.compute_alpha(local_frame_acceleration)
        # complete LERP or SLERP
        if delta_Mag.q0 > self.epsilon:
            lerp_quat = (1- weight) * self.unit_quat + weight * delta_Mag
            lerp_quat = lerp_quat.normalize()
            print("lerp")
            print(lerp_quat)
            print("Result before mag lerp")
            print(self.qResult)
            self.qResult = self.qResult * lerp_quat
        else:  
            # in progress
            qIdentity = Quaternion()
            angle = math.acos(delta_Mag.q0)

            slerp_quat =(math.sin((1 - weight) * angle) / math.sin(angle)) * qIdentity + (math.sin(weight * angle) / math.sin(angle)) * delta_Mag
            print("slerp")
            print(slerp_quat)
            print("Result before mag slerp")
            print(self.qResult)
            self.qResult = self.qResult * slerp_quat
        print("Result after mag lerp/slerp")
        print(self.qResult)

    """
    Correction step.

    Because each delta quaternion is computed and filtered independently,
    they are separated into two steps.
    First should be accelerometer-based correction, then magnetometer-based correction.
    """
    def correctOrientation(self, local_frame_acceleration:Vector, magnetometer:Vector):
        self.correctAcceleration(local_frame_acceleration)
        self.correctMagneticField(magnetometer)
        self.qEstimate = self.qResult

    def graphResult(self, imu, mag, vAcc, vGyro, vMag):
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        vector = np.array([0, 0, 0])

        def animate(i):
            # read file and draw vectors using quiver
            # all start at 0,0,0
            ax.clear()
            
            vAcc.x = imu.get_acceleration('x')
            vAcc.y = imu.get_acceleration('y')
            vAcc.z = imu.get_acceleration('z')

            vMag.set(*mag.get_magnetic())


            vGyro.x = imu.get_gyroscope('x')
            vGyro.y = imu.get_gyroscope('y')
            vGyro.z = imu.get_gyroscope('z')
            
            self.currTime = time.time()

            # Normalize vectors for orientation sensor fusion
            vNormAccel = vAcc.normalize()
            vNormMag = vMag.normalize()

            # Prediction Step
            self.predict(vGyro)

            # Correction Step
            self.correctOrientation(vNormAccel, vNormMag)
                    
            axis, angle = self.toAxisAngle(self.qResult)

            vector[0] = axis.x
            vector[1] = axis.y
            vector[2] = axis.z

            for i in range(0,3):
                vector[i] = float(vector[i])

            x = np.array([1.0,0.0,0.0])  # take a unit x vector
            x[0] = x[0] + math.cos(angle) * math.sqrt(2) #x and y adjustment from angle
            x[1] = x[1] + math.sin(angle) * math.sqrt(2)
            # x[0] = x[0] + math.sqrt(2) * math.cos(angle)
            dot_prod = np.dot(x,vector)
            for i in range(0,3):
                x[i] = x[i] - ((dot_prod * vector[i]) / np.linalg.norm(vector)**2)
            x = x / np.linalg.norm(x)  # normalize it
            y = np.cross(vector, x)      # cross product with k
            # x[0] = x[0] + math.cos(angle) * math.sqrt(2)
            # x[1] = x[1] + math.sin(angle) * math.sqrt(2)
            # y[0] = y[0] - math.sin(angle) * math.sqrt(2)
            # y[1] = y[1] - math.cos(angle) * math.sqrt(2)
            # print(vector)
            # print(x)
            # print(y)

            vec1 = ax.quiver(0,0,0,vector[0],vector[1],vector[2],color='blue')
            vec2 = ax.quiver(0,0,0,x[0],x[1],x[2],color='red')
            vec3 = ax.quiver(0,0,0,y[0],y[1],y[2],color='green')

            # Draw x and y lists
            ax.set_xlim([-2,2])
            ax.set_ylim([-2,2])
            ax.set_zlim([-2,2])

            # Format plot
            # plt.xticks(rotation=45, ha='right')
            # plt.subplots_adjust(bottom=0.30)
            plt.title('Axis Angle Representation over Time')
            # plt.ylabel('Top to bottom: Roll, Pitch, Yaw')
            return vec1, vec2, vec3
        # Set up plot to call animate() function periodically
        ani = animation.FuncAnimation(fig, animate)
        plt.show()
    
    def toEuler(self, q:Quaternion):
        angles = Vector()

        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.q0 * q.q1 + q.q2 * q.q3)
        cosr_cosp = 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2)
        angles.x = math.atan2(sinr_cosp, cosr_cosp) * 180 / math.pi

        # pitch (y-axis rotation)
        sinp = math.sqrt(math.fabs(1 + 2 * (q.q0 * q.q2 - q.q1 * q.q3)))
        cosp = math.sqrt(math.fabs(1 - 2 * (q.q0 * q.q2 - q.q1 * q.q3)))
        angles.y = (2 * math.atan2(sinp, cosp) - math.pi / 2) * 180 / math.pi

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.q0 * q.q3 + q.q1 * q.q2)
        cosy_cosp = 1 - 2 * (q.q2 * q.q2 + q.q3 * q.q3)
        angles.z = math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi

        if(angles.z < 0):
            angles.z += 360

        return angles

    def toAxisAngle(self, q:Quaternion):
        axis = Vector()
        q = q.normalize()
        
        angle = 2 * math.acos(q.q0)
        denom = math.sqrt(1- (q.q0 * q.q0))
        if denom < 0.001:
            axis.x = q.q1
            axis.y = q.q2
            axis.z = q.q3
        else:
            axis.x = q.q1 / denom
            axis.y = q.q2 / denom
            axis.z = q.q3 / denom

        return axis,angle
