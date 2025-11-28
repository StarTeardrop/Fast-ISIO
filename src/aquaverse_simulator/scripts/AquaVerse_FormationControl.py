from AquaUDPReceiver import AquaReceiverStart
from AquaUDPTransporter import *
import time
import threading
import csv
import os
from PacketParse import *


def handle_dvl(msg):
    dvl_data = parse_dvl_packet(msg)
    print(f"DVl Velocity: x_vel:{dvl_data['Velocity']['x']}, y_vel:{dvl_data['Velocity']['y']}")


if __name__ == '__main__':
    on_topic_callbacks = {
        '/BlueRov2/0/Dvl': handle_dvl,
    }

    receiver_thread = threading.Thread(target=AquaReceiverStart, args=(on_topic_callbacks,))
    receiver_thread.start()

    while True:
        robots = [
            SendMessage(10, 10, 10, 10, -0, -0, -0, -0, enum_robots['BlueRov2'], 0),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 1),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 2),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 3),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 4),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 5),
            SendMessage(10, 10, 10, 10, 0, 0, 0, 0, enum_robots['BlueRov2'], 6),
        ]
        control_multi_agents(robots)
        time.sleep(0.01)
