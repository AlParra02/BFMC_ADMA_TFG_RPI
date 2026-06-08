import struct

COMM_GET_IMU_DATA = 65

def build_imu_request():
    """Build a COMM_GET_IMU_DATA request with mask=0xFFFF (all fields)."""
    payload = bytes([COMM_GET_IMU_DATA, 0xFF, 0xFF])
    length  = len(payload)
    crc     = _crc16(payload)
    # small frame: [0x02, length, ...payload..., crc_hi, crc_lo, 0x03]
    return bytes([0x02, length]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

def parse_imu_response(data):
    """Parse the COMM_GET_IMU_DATA response payload (after stripping framing)."""
    i = 0
    roll, pitch, yaw = struct.unpack_from('>fff', data, i); i += 12
    ax, ay, az       = struct.unpack_from('>fff', data, i); i += 12
    gx, gy, gz       = struct.unpack_from('>fff', data, i); i += 12
    mx, my, mz       = struct.unpack_from('>fff', data, i); i += 12
    q0, q1, q2, q3   = struct.unpack_from('>ffff', data, i)
    return {
        'rpy': (roll, pitch, yaw),
        'accel': (ax, ay, az),     # m/s² (already scaled in firmware)
        'gyro': (gx, gy, gz),      # rad/s
        'mag': (mx, my, mz),
        'quat': (q0, q1, q2, q3),
    }

def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF