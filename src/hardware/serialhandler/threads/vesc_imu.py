# vesc_imu.py
#
# Defines COMM_GET_IMU_DATA (id=65) as a proper pyvesc message class using
# the VESCMessage metaclass, so pyvesc's own encode/decode machinery
# (framing, CRC, buffer handling) takes care of this packet just like any
# built-in pyvesc message (GetValues, SetDutyCycle, etc).
#
# Import this module once anywhere before calling pyvesc.encode_request() /
# pyvesc.decode() for IMU data — the metaclass registers the class into
# VESCMessage._msg_registry automatically on class definition.

from pyvesc.messages.base import VESCMessage


class GetImuData(metaclass=VESCMessage):
    """COMM_GET_IMU_DATA request/response (VESC command id = 65).

    Request:
        GetImuData() with no field values — pyvesc.encode_request(GetImuData)
        sends just the id byte (header_only pack), which matches the request
        format used by VESC Tool to ask for IMU data.

    Response fields (16 big-endian floats after the id byte):
        roll, pitch, yaw   : degrees
        acc_x, acc_y, acc_z: m/s^2
        gyro_x, gyro_y, gyro_z: deg/s  (firmware sends deg/s, not rad/s)
        mag_x, mag_y, mag_z: raw magnetometer units
        q0, q1, q2, q3     : unit quaternion components

    NOTE: If your firmware build sends a different field set (e.g. some
    firmwares omit magnetometer fields, or send a leading 'mask' field
    echoing the request), this 'fields' list must be adjusted to match —
    otherwise struct.unpack will raise "unpack requires a buffer of N bytes"
    and pyvesc.decode() will return None / raise, which is the same class of
    bug as our original hand-rolled frame.

    To verify your firmware's exact layout, see commands.c /
    datatypes.h in the bldc firmware source matching
    'VESC Tool -> Firmware -> Connected VESC's Firmware' version.
    """
    id = 65
    fields = [
        ('roll',  'f'),
        ('pitch', 'f'),
        ('yaw',   'f'),
        ('acc_x', 'f'),
        ('acc_y', 'f'),
        ('acc_z', 'f'),
        ('gyro_x','f'),
        ('gyro_y','f'),
        ('gyro_z','f'),
        ('mag_x', 'f'),
        ('mag_y', 'f'),
        ('mag_z', 'f'),
        ('q0',    'f'),
        ('q1',    'f'),
        ('q2',    'f'),
        ('q3',    'f'),
    ]
