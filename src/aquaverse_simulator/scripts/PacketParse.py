import struct


def parse_imu_packet(msg: dict):
    data = msg['data']
    fmt = '<10f'  # 10 floats, small end sequence
    expected_size = struct.calcsize(fmt)  # Should be 40 bytes
    if len(data) != expected_size:
        raise ValueError(f"Invalid data length: expected {expected_size}, got {len(data)}")

    unpacked = struct.unpack(fmt, data)
    imu_data = {
        'Acceleration': {
            'x': unpacked[0],
            'y': unpacked[1],
            'z': unpacked[2],
        },
        'AngularVelocity': {
            'x': unpacked[3],
            'y': unpacked[4],
            'z': unpacked[5],
        },
        'Orientation': {
            'w': unpacked[6],
            'x': unpacked[7],
            'y': unpacked[8],
            'z': unpacked[9],
        },
    }
    return imu_data


def parse_dvl_packet(msg: dict):
    data = msg['data']
    fmt = '<3f'  # Three floats, small end sequence
    expected_size = struct.calcsize(fmt)  # 12 Bytes
    if len(data) != expected_size:
        raise ValueError(f"Invalid data length: expected {expected_size}, got {len(data)}")

    unpacked = struct.unpack(fmt, data)
    dvl_data = {
        'Velocity': {
            'x': unpacked[0],
            'y': unpacked[1],
            'z': unpacked[2],
        }
    }
    return dvl_data


def parse_position_packet(msg: dict):
    data = msg['data']
    fmt = '<10f'  # 10 floats, small end sequence
    expected_size = struct.calcsize(fmt)  # 52 Bytes
    if len(data) != expected_size:
        raise ValueError(f"Invalid data length: expected {expected_size}, got {len(data)}")

    unpacked = struct.unpack(fmt, data)
    position_data = {
        'Position': {
            'x': unpacked[0],
            'y': unpacked[1],
            'z': unpacked[2],
        },
        'OrientationEuler': {
            'pitch': unpacked[3],
            'roll': unpacked[4],
            'yaw': unpacked[5],
        },
        'OrientationQuat': {
            'w': unpacked[6],
            'x': unpacked[7],
            'y': unpacked[8],
            'z': unpacked[9],
        }
    }
    return position_data
