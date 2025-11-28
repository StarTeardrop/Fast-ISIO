#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
import numpy as np
import math
import time
from AquaUDPTransporter import *

target_vel = np.zeros(3)   # [vx, vy, wz]
current_vel = np.zeros(3)  # [vx, vy, wz]

last_time = None
integral_error = np.zeros(3)

dvl_ready = False 


Kp = np.array([750, 500, 18])
Ki = np.array([500, 0, 1])
Kd = np.array([2, 0, 0.0])


INTEGRAL_LIMIT = np.array([0.2, 0.2, 0.05]) 

prev_error = np.zeros(3)


def imu_callback(msg: Imu):
    global current_vel
    current_vel[2] = msg.angular_velocity.z
    

def dvl_callback(msg: TwistStamped):
    global current_vel, dvl_ready
    current_vel[0] = msg.twist.linear.x
    current_vel[1] = msg.twist.linear.y

    dvl_ready = True


def cmd_vel_callback(msg: TwistStamped):
    global target_vel

    if not dvl_ready:
        return

    target_vel[0] = msg.twist.linear.x
    target_vel[1] = msg.twist.linear.y
    target_vel[2] = msg.twist.angular.z

    force_vector = pid_control() 

    # print(f"force_vector: {force_vector}")

    thruster_cmds  = Thrust_Matrix_pinv @ force_vector
    thruster_cmds = np.clip(thruster_cmds, -50, 50)

    # print(f"thruster_cmds: {thruster_cmds}")

    robots = [
            SendMessage(thruster_cmds[0], thruster_cmds[1], thruster_cmds[2], thruster_cmds[3],
                        0, 0, 0, 0, enum_robots['BlueRov2'], 0),
        ]
    control_multi_agents(robots)


def pid_control():
    global prev_error, integral_error, last_time
    now = time.time()
    if last_time is None:
        last_time = now
        return np.zeros(3)
    dt = now - last_time
    last_time = now

    error = target_vel - current_vel

    derivative = (error - prev_error) / dt if dt > 0 else np.zeros(3)

    for i in range(3):
        if abs(error[i]) < INTEGRAL_LIMIT[i]:
            integral_error[i] += error[i] * dt

    prev_error = error

    output = Kp * error + Ki * integral_error + Kd * derivative
    output[1] = -output[1]  
    output[2] = -output[2] 
    return output


if __name__ == '__main__':
    rospy.init_node('speed_horizontal_attitude_control_node')

    rospy.loginfo("\033[1;32m----> Auqaverse Speed Horizontal Attitude Control Node Started.\033[0m")


    rospy.Subscriber('/BlueRov2/0/dvl', TwistStamped, dvl_callback)

    rospy.Subscriber('/BlueRov2/0/imu', Imu, imu_callback)

    rospy.Subscriber('/BlueRov2/0/cmd_vel', TwistStamped, cmd_vel_callback)


    alpha = 45
    cos_alpha = math.cos(math.radians(alpha))
    sin_alpha = math.sin(math.radians(alpha))
    c = 0.24
    Thrust_Matrix = np.array([[cos_alpha, cos_alpha, cos_alpha, cos_alpha],
                            [sin_alpha, -sin_alpha, -sin_alpha, sin_alpha],
                            [c * sin_alpha, -c * sin_alpha, c * sin_alpha, -c * sin_alpha]])

    Thrust_Matrix_pinv = np.linalg.pinv(Thrust_Matrix)

    rospy.spin()

