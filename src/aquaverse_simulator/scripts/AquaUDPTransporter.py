import struct
import socket
from typing import List

FORMAT_HEAD1 = 0x0A
FORMAT_HEAD2 = 0x0B
FORMAT_TAIL1 = 0x0C
FORMAT_TAIL2 = 0x0D

enum_robots = {
    'BlueRov2': 0,
    'Other': 1,
}

class SendMessage:
    def __init__(self, t1, t2, t3, t4, t5, t6, t7, t8, robot_type, robot_id):
        self.thrusts = [t1, t2, t3, t4, t5, t6, t7, t8]
        self.robot_type = robot_type
        self.robot_id = robot_id


def pack_multi_message(messages: List[SendMessage]) -> bytes:
    header = bytes([FORMAT_HEAD1, FORMAT_HEAD2])
    tail = bytes([FORMAT_TAIL1, FORMAT_TAIL2])

    count = len(messages)
    body = struct.pack('<B', count)

    for msg in messages:
        body += struct.pack('<8fBB', *msg.thrusts, msg.robot_type, msg.robot_id)

    return header + body + tail

def control_multi_agents(messages: List[SendMessage], ip='192.168.3.25', port=6666):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = pack_multi_message(messages)
    sock.sendto(packet, (ip, port))
