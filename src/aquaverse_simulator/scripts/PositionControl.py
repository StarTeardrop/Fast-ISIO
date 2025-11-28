#! /usr/bin/env python
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np
import math
import time
from AquaUDPTransporter import *

class ControlState:
    TRANSLATING = 0  
    ROTATING = 1    
    COMPLETED = 2   

class PID:
    def __init__(self, Kp, Ki, Kd, limit):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit
        self.integral = 0
        self.last_error = 0

    def reset(self):
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        if dt <= 0:
            return 0
        
        if abs(error) < self.limit:  
            self.integral += error * dt

        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative 
        return output


current_pos = np.zeros(3)
target_pos = np.zeros(3)
target_received = False
control_state = ControlState.TRANSLATING 
last_time = None


pid_x_translate = PID(Kp=6.3484445, Ki=0.1, Kd=0.0, limit=1.0)
pid_y_translate = PID(Kp=6, Ki=0.05, Kd=0.0, limit=1.0)
pid_yaw_translate = PID(Kp=6, Ki=0.05, Kd=0.05, limit=1.0)  


pid_x_rotate = PID(Kp=0.5, Ki=0.01, Kd=0.1, limit=0.5)  
pid_y_rotate = PID(Kp=0.2, Ki=0.01, Kd=0.1, limit=0.5)
pid_yaw_rotate = PID(Kp=3, Ki=0.05, Kd=0.05, limit=1) 


POSITION_TOLERANCE = 0.2  
YAW_TOLERANCE = 0.01      
STABLE_TIME_THRESHOLD = 0.5  
current_stable_time = 0.0   


def angle_diff(target, current):
    diff = target - current
    return (diff + math.pi) % (2 * math.pi) - math.pi

def position_callback(msg: Odometry):
    global current_pos
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    current_pos[0] = pos.x
    current_pos[1] = pos.y
    quat = [ori.x, ori.y, ori.z, ori.w]
    roll, pitch, yaw = euler_from_quaternion(quat)
    current_pos[2] = yaw

    # print("current_pos: ", current_pos)

    update_control()


def control_position_callback(msg: Odometry):
    global target_pos, target_received, control_state, current_stable_time

    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    new_target_pos = np.array([
        pos.x,
        pos.y,
        euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]  # yaw角度
    ])
    
    position_changed = not np.allclose(target_pos[:2], new_target_pos[:2], atol=0.01)
    orientation_changed = abs(target_pos[2] - new_target_pos[2]) > 0.01  # 约0.57度
    
    if not position_changed and not orientation_changed:
        # rospy.loginfo("New target is the same as current target. Ignoring.")
        return
    
    target_pos = new_target_pos
    target_received = True
    control_state = ControlState.TRANSLATING  
    current_stable_time = 0.0
    
    pid_x_translate.reset()
    pid_y_translate.reset()
    pid_yaw_translate.reset()
    pid_x_rotate.reset()
    pid_y_rotate.reset()
    pid_yaw_rotate.reset()
    
    # rospy.loginfo("New target received: (%.2f, %.2f, %.1f°)", target_pos[0], target_pos[1], math.degrees(target_pos[2]))
    # rospy.loginfo("Starting translation to target position...")


def update_control():
    global last_time, control_state, current_stable_time
    
    if not target_received:
        return
        
    now = rospy.Time.now().to_sec()
    if last_time is None:
        last_time = now
        return
        
    dt = now - last_time
    last_time = now
    
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    position_error = math.sqrt(dx*dx + dy*dy)
    
    dyaw = angle_diff(target_pos[2], current_pos[2])

    # print(f"position_error: {position_error}, dyaw: {dyaw}")

    if dyaw > math.pi:
        dyaw -= 2 * math.pi
    elif dyaw < -math.pi:
        dyaw += 2 * math.pi

    # print(f"target yaw: {target_pos[2]}, current yaw: {current_pos[2]},  yaw error: {dyaw}")
    

    if control_state == ControlState.TRANSLATING:
        if position_error < POSITION_TOLERANCE:
            current_stable_time += dt
            # if current_stable_time >= STABLE_TIME_THRESHOLD:
            control_state = ControlState.ROTATING
                # current_stable_time = 0.0
                # rospy.loginfo("Translation completed. Starting rotation to target orientation...")
        else:
            current_stable_time = 0.0
            
        vx_world = pid_x_translate.compute(dx, dt)
        vy_world = pid_y_translate.compute(dy, dt)

        wz = 0
        
    elif control_state == ControlState.ROTATING:
        if abs(dyaw) < YAW_TOLERANCE:
            current_stable_time += dt
            if current_stable_time >= STABLE_TIME_THRESHOLD:
                control_state = ControlState.COMPLETED
                current_stable_time = 0.0
                rospy.loginfo("Rotation completed. Target reached!")
        else:
            current_stable_time = 0.0
            
        vx_world = pid_x_rotate.compute(dx, dt)  
        vy_world = pid_y_rotate.compute(dy, dt)
        wz = pid_yaw_rotate.compute(dyaw, dt)     
        
    else:  # ControlState.COMPLETED
        vx_world = pid_x_rotate.compute(dx, dt)
        vy_world = pid_y_rotate.compute(dy, dt)
        wz = pid_yaw_rotate.compute(dyaw, dt)
    
    yaw = current_pos[2]
    vx_body = math.cos(-yaw) * vx_world - math.sin(-yaw) * vy_world
    vy_body = math.sin(-yaw) * vx_world + math.cos(-yaw) * vy_world

    vy_body = -vy_body  
    wz = -wz 

    cmd = np.array([vx_body, vy_body, wz])
    
    thrusts = Thrust_Matrix_pinv @ cmd
    thruster_cmds = np.clip(thrusts, -50, 50)

    # print("thruster_cmds: ", thruster_cmds)

    robots = [
        SendMessage(thruster_cmds[0], thruster_cmds[1], thruster_cmds[2], thruster_cmds[3],
                    0, 0, 0, 0, enum_robots['BlueRov2'], 0),
    ]
    control_multi_agents(robots)


if __name__ == '__main__':
    rospy.init_node('position_control_node')

    rospy.loginfo("\033[1;32m---->  AquaVerse Position Control Node Started.\033[0m")

    rospy.Subscriber('/BlueRov2/0/position', Odometry, position_callback)
    rospy.Subscriber('/BlueRov2/0/position_control', Odometry, control_position_callback)


    alpha = 45
    cos_alpha = math.cos(math.radians(alpha))
    sin_alpha = math.sin(math.radians(alpha))
    c = 0.24
    Thrust_Matrix = np.array([[cos_alpha, cos_alpha, cos_alpha, cos_alpha],
                            [sin_alpha, -sin_alpha, -sin_alpha, sin_alpha],
                            [c * sin_alpha, -c * sin_alpha, c * sin_alpha, -c * sin_alpha]])
    Thrust_Matrix_pinv = np.linalg.pinv(Thrust_Matrix)

    rospy.spin()


    