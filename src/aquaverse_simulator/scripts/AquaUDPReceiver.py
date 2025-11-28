import socket
import struct
import threading
import time
import os

# -------------------------
# Define parsing header format, corresponding to c++
HEADER_FORMAT = '<Q I d H H I I B B'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 30 bytes


# Parse header
def parse_header(data: bytes):
    if len(data) < HEADER_SIZE:
        raise ValueError("Insufficient packet length to parse header")
    unpacked = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
    MsgID = unpacked[0]
    MsgType = unpacked[1]
    Timestamp = unpacked[2]
    FragmentIndex = unpacked[3]
    FragmentCount = unpacked[4]
    FragmentSize = unpacked[5]
    FullDataSize = unpacked[6]
    IsFirst = unpacked[7]
    IsLast = unpacked[8]

    return {
        'MsgID': MsgID,
        'MsgType': MsgType,
        'Timestamp': Timestamp,
        'FragmentIndex': FragmentIndex,
        'FragmentCount': FragmentCount,
        'FragmentSize': FragmentSize,
        'FullDataSize': FullDataSize,
        'IsFirst': IsFirst,
        'IsLast': IsLast
    }


# Parsing a single UDP packet
def parse_packet(packet: bytes):
    header = parse_header(packet)
    offset = HEADER_SIZE

    topic = ''
    image_name = ''

    if header['IsFirst']:
        if len(packet) < offset + 1:
            raise ValueError("The packet is too short to read the TopicName length")
        topic_len = packet[offset]
        offset += 1

        if len(packet) < offset + topic_len + 1:
            raise ValueError("The packet is too short to read the TopicName string")
        topic = packet[offset:offset + topic_len].decode('utf-8')
        offset += topic_len

        image_name_len = packet[offset]
        offset += 1

        if len(packet) < offset + image_name_len:
            raise ValueError("The packet is too short to read the ImageName string")
        image_name = packet[offset:offset + image_name_len].decode('utf-8')
        offset += image_name_len

    fragment_data = packet[offset:offset + header['FragmentSize']]

    return header, topic, image_name, fragment_data

# -------------------------
# Cache all fragments and wait for reassembly
class FragmentBuffer:
    def __init__(self):
        self.buffers = {}
        self.complete_messages = {}

    def add_fragment(self, header, topic, message_name, data):
        msgid = header['MsgID']

        if msgid not in self.buffers:
            self.buffers[msgid] = {
                'fragments': {},
                'count': header['FragmentCount'],
                'received': 0,
                'full_size': header['FullDataSize'],
                'topic': topic,
                'message_name': message_name,
                'timestamp': header['Timestamp'],
                'msg_type': header['MsgType'],
                'last_update': time.time()
            }

        buf = self.buffers[msgid]
        if header['FragmentIndex'] not in buf['fragments']:
            buf['fragments'][header['FragmentIndex']] = data
            buf['received'] += 1
            buf['last_update'] = time.time()

    def is_complete(self, msgid):
        return msgid in self.buffers and self.buffers[msgid]['received'] == self.buffers[msgid]['count']

    def assemble(self, msgid):
        buf = self.buffers.get(msgid)
        if not buf:
            return None
        fragments = buf['fragments']
        full_data = b''.join(fragments[i] for i in range(buf['count']))
        if len(full_data) != buf['full_size']:
            print(f"Warning: restructured data length mismatch: {len(full_data)} != {buf['full_size']}")
        result = {
            'topic': buf['topic'],
            'message_name': buf['message_name'],
            'timestamp': buf['timestamp'],
            'msg_type': buf['msg_type'],
            'data': full_data
        }
        self.complete_messages[result['topic']] = result
        return result

    def get(self, topic_name):
        return self.complete_messages.get(topic_name)

    def remove(self, msgid):
        if msgid in self.buffers:
            topic = self.buffers[msgid]['topic']
            if topic in self.complete_messages:
                del self.complete_messages[topic]
            del self.buffers[msgid]

    def cleanup_expired(self, timeout=10):
        now = time.time()
        expired = [msgid for msgid, buf in self.buffers.items() if now - buf['last_update'] > timeout]
        for msgid in expired:
            print(f"Cleanup timeout incomplete message {msgid}")
            self.remove(msgid)

# -------------------------
# UDP listener thread
def udp_listen(ip='0.0.0.0', port=12345, frag_buffer=None, on_message_dict=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    print(f"Listening for UDP {ip}:{port}")

    if frag_buffer is None:
        frag_buffer = FragmentBuffer()

    if on_message_dict is None:
        on_message_dict = {}

    while True:
        packet, addr = sock.recvfrom(65535)
        try:
            header, topic, image_name, fragment_data = parse_packet(packet)
        except Exception as e:
            print(f"Parsing failed: {e}")
            continue

        frag_buffer.add_fragment(header, topic, image_name, fragment_data)

        if frag_buffer.is_complete(header['MsgID']):
            full_msg = frag_buffer.assemble(header['MsgID'])
            # print(f"Complete message received，MsgID={header['MsgID']}")
            # print(f"Topic: {full_msg['topic']}, MessageName: {full_msg['message_name']}, Timestamp: {full_msg['timestamp']}")
            # print(f"data size: {len(full_msg['data'])} bytes, type: {full_msg['msg_type']}")

            # ✅ Call callback function to process complete data
            topic = full_msg['topic']
            # ✅ Distribute callback according to topic
            if topic in on_message_dict:
                try:
                    on_message_dict[topic](full_msg)
                except Exception as e:
                    print(f"[Callback error] handle topic={topic} Error: {e}")

            frag_buffer.remove(header['MsgID'])

        frag_buffer.cleanup_expired()

def AquaReceiverStart(on_topic_callbacks: dict):
    frag_buffer = FragmentBuffer()
    udp_thread = threading.Thread(
        target=udp_listen,
        args=('192.168.3.2', 1516, frag_buffer),
        kwargs={'on_message_dict': on_topic_callbacks},
        daemon=True
    )
    udp_thread.start()
