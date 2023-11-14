""" IMPORT STATEMENTS """
import math
from SensorFusion.Quaternion import Quaternion
from SensorFusion.Vector import Vector

def orientation(accel_data:Vector, mag_data:Vector = None):
    qAccelerometer = Quaternion()
    qMagnetometer = Quaternion()
    if(mag_data is None):
        qResult = Quaternion()
        p_r_y = accel_data
        # calculate the quaternion components based on euler angle trig
        # note: function assumes angles in radians
        qResult.set_q0(math.cos(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.cos(p_r_y.z/2)
                       + math.sin(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q1(math.sin(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.cos(p_r_y.z/2)
                        - math.cos(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q2(math.cos(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.cos(p_r_y.z/2)
                        + math.sin(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.sin(p_r_y.z)/2)
        qResult.set_q3(math.cos(p_r_y.y/2) * math.cos(p_r_y.x/2) * math.sin(p_r_y.z/2)
                       - math.sin(p_r_y.y/2) * math.sin(p_r_y.x/2) * math.cos(p_r_y.z)/2)

        return qResult
    # calculate the quaternion components based on euler angle trig
    # note: function assumes angles in radians
    if accel_data.z >= 0:
        qAccelerometer.set_q0(math.sqrt((accel_data.z +1)/2))
        qAccelerometer.set_q1(-1*(accel_data.y)/math.sqrt(2*(accel_data.z +1)))
        qAccelerometer.set_q2((accel_data.x)/math.sqrt(2*(accel_data.z +1)))
        qAccelerometer.set_q3(0)
    else:
        qAccelerometer.set_q0(-1*(accel_data.y)/math.sqrt(2*(1-accel_data.z)))
        qAccelerometer.set_q1(math.sqrt((1-accel_data.z)/2))
        qAccelerometer.set_q2(0)
        qAccelerometer.set_q3((accel_data.x)/math.sqrt(2*(1-accel_data.z)))

    rotatedMagField = Quaternion.rotateTMultiply(qAccelerometer, mag_data)
    gamma = rotatedMagField.x**2 + rotatedMagField.y**2

    if rotatedMagField.x >= 0:
        qMagnetometer.set_q0(math.sqrt((gamma + rotatedMagField.x * math.sqrt(gamma))/(2*gamma)))
        qMagnetometer.set_q1(0)
        qMagnetometer.set_q2(0)
        qMagnetometer.set_q3(rotatedMagField.y/(math.sqrt(2*(gamma + rotatedMagField.x * math.sqrt(gamma)))))
    else:
        qMagnetometer.set_q0(rotatedMagField.y/(math.sqrt(2*(gamma - rotatedMagField.x * math.sqrt(gamma)))))
        qMagnetometer.set_q1(0)
        qMagnetometer.set_q2(0)
        qMagnetometer.set_q3(math.sqrt((gamma - rotatedMagField.x * math.sqrt(gamma))/(2*gamma)))

    return qAccelerometer, qMagnetometer
    
# Find the current position & velocity of the object
def currentPositionVelocity(acceleration, quaternion, prevVel, prevPos, timeDelta):
    gravity = Vector()
    gravity.x = 2 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2) * 9.8
    gravity.y = 2 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3) * 9.8
    gravity.z = 9.8 * (quaternion.q0**2 - quaternion.q1**2 - quaternion.q2**2 + quaternion.q3**2)

    linearAccel = Vector()

    linearAccel.x = acceleration.x - gravity.x
    linearAccel.y = acceleration.y - gravity.y
    linearAccel.z = acceleration.z - gravity.z

    velocity = Vector()

    velocity.x = prevVel.x + linearAccel.x * timeDelta
    velocity.y = prevVel.y + linearAccel.y * timeDelta
    velocity.z = prevVel.z + linearAccel.z * timeDelta

    position = Vector()

    position.x = prevPos.x + prevVel.x * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.x
    position.y = prevPos.y + prevVel.y * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.y
    position.z = prevPos.z + prevVel.z * timeDelta + ((timeDelta * timeDelta)/2) * linearAccel.z

    return position, velocity
