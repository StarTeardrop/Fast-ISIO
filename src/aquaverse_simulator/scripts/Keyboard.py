import time
import math
import numpy as np
from threading import Thread
from pynput import keyboard
from AquaUDPReceiver import AquaReceiverStart
from AquaUDPTransporter import *
import sys
import termios
import tty

alpha = 45
cos_alpha = math.cos(math.radians(alpha))
sin_alpha = math.sin(math.radians(alpha))
c = 0.24
Thrust_Matrix = np.array([[cos_alpha, cos_alpha, cos_alpha, cos_alpha],
                          [sin_alpha, -sin_alpha, -sin_alpha, sin_alpha],
                          [c * sin_alpha, -c * sin_alpha, c * sin_alpha, -c * sin_alpha]])
# Pseudo inverse matrix
Thrust_Matrix_pinv = np.linalg.pinv(Thrust_Matrix)

Torch_X = 4
Torch_Y = 8
Torch_Yaw = 1
Torch_Z_Down = 50
Torch_Z_Up = 15

# Current control target vector
target_force = np.array([0.0, 0.0, 0.0])  # [Fx, Fy, Tau_z]
vertical_thrust = 0.0  # Vertical thrust

# Lock and run flag
running = True


def send_control_loop():
    global vertical_thrust
    while running:
        thrusters = Thrust_Matrix_pinv @ target_force  # Thrust of 4 horizontal thrusters
        # The thrust of vertical thrusters is the same, and the vertical upward is positive, and the downward is negative
        robots = [
            SendMessage(thrusters[0], thrusters[1], thrusters[2], thrusters[3],
                        vertical_thrust, vertical_thrust, vertical_thrust, vertical_thrust, enum_robots['BlueRov2'], 0),
        ]

        control_multi_agents(robots)
        time.sleep(0.01)  # 1000Hz， It can also be lowered


def on_press(key):
    global target_force, vertical_thrust
    try:
        if key.char == 'i':
            target_force[0] = Torch_X
        elif key.char == 'k':
            target_force[0] = -Torch_X
        elif key.char == 'l':
            target_force[1] = Torch_Y
        elif key.char == 'j':
            target_force[1] = -Torch_Y
        elif key.char == 'u':
            target_force[2] = -Torch_Yaw
        elif key.char == 'o':
            target_force[2] = Torch_Yaw
        elif key.char == 'x':  # Up
            vertical_thrust = Torch_Z_Up
        elif key.char == 'z':  # Down
            vertical_thrust = -Torch_Z_Down
    except AttributeError:
        pass


def on_release(key):
    global target_force, vertical_thrust
    try:
        if key.char in ['i', 'k']:
            target_force[0] = 0
        elif key.char in ['j', 'l']:
            target_force[1] = 0
        elif key.char in ['u', 'o']:
            target_force[2] = 0
        elif key.char in ['x', 'z']:
            vertical_thrust = 0
    except AttributeError:
        if key == keyboard.Key.esc:
            print("ESC pressed, exiting...")
            return False  # Exit listening
        

class TerminalSettings:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)

    def disable_echo(self):
        tty.setcbreak(self.fd)
        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] = new_settings[3] & ~termios.ECHO  # lflags
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new_settings)

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
