# vesc_imu.py
#
# Self-contained COMM_GET_IMU_DATA (command id = 65) request builder and
# response parser for VESC firmware 6.x / 7.x.
#
# Replaces BOTH the previous vesc_imu.py (pyvesc VESCMessage approach) and
# COMM_GET_IMU_DATA.py. We do NOT use pyvesc for the IMU because:
#   * the RESPONSE payload is [id][mask_hi][mask_lo][floats...] — the 2-byte
#     mask sits between the id and the floats, so a plain field list misaligns
#     every value by 2 bytes;
#   * the REQUEST must carry a 2-byte mask, but pyvesc.encode_request() emits
#     only the id byte, so the firmware reads mask=0 and returns no floats.
#
# This module frames/parses both sides explicitly and is mask-aware, so it
# also works on 6-axis IMUs (the magnetometer bits simply won't be set).

import struct

# ── Protocol constants ────────────────────────────────────────────────────────
COMM_GET_IMU_DATA = 65
_FRAME_START_SHORT = 0x02   # payloads < 256 bytes
_FRAME_END = 0x03

# Full mask: request every field the firmware can supply.
IMU_MASK_ALL = 0xFFFF

# Field order as emitted by bldc commands.c (one float per set mask bit, in
# ascending bit order). This order is stable across FW 6.x / 7.x.
IMU_FIELD_ORDER = [
    "roll", "pitch", "yaw",          # bits 0-2   : degrees
    "acc_x", "acc_y", "acc_z",       # bits 3-5   : g  (multiply by 9.80665 for m/s^2)
    "gyro_x", "gyro_y", "gyro_z",    # bits 6-8   : deg/s (multiply by pi/180 for rad/s)
    "mag_x", "mag_y", "mag_z",       # bits 9-11  : raw magnetometer units
    "q0", "q1", "q2", "q3",          # bits 12-15 : unit quaternion
]

# Optional unit conversions (apply downstream if you need SI units).
G_TO_MS2 = 9.80665
DEG_TO_RAD = 3.141592653589793 / 180.0


# ── CRC-16 (XMODEM: poly 0x1021, init 0x0000, no reflection) ───────────────────
def crc16(data: bytes) -> int:
    """VESC packet CRC. Computed over the payload only (id + mask + floats)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
    return crc & 0xFFFF


# ── Request ────────────────────────────────────────────────────────────────────
def build_imu_request(mask: int = IMU_MASK_ALL) -> bytes:
    """Build a framed COMM_GET_IMU_DATA request.

    Frame: [0x02][len][65][mask_hi][mask_lo][crc_hi][crc_lo][0x03]

    Args:
        mask: 16-bit field selector. 0xFFFF requests all fields.

    Returns:
        Fully framed, CRC'd bytes ready to write to the serial port.
    """
    payload = bytes([COMM_GET_IMU_DATA, (mask >> 8) & 0xFF, mask & 0xFF])
    crc = crc16(payload)
    return (
        bytes([_FRAME_START_SHORT, len(payload)])
        + payload
        + bytes([(crc >> 8) & 0xFF, crc & 0xFF, _FRAME_END])
    )


# ── Servo / steering (COMM_SET_SERVO_POS, id 12) ──────────────────────────────
COMM_SET_SERVO_POS = 12  # 0x0C  (NOT 33 — that is COMM_GET_DECODED_CHUK)


def build_servo_command(pos: float) -> bytes:
    """Build a framed COMM_SET_SERVO_POS command.

    Frame: [0x02][len][33][pos_hi][pos_lo][crc_hi][crc_lo][0x03]
    The servo position is an int16 = round(pos * 1000), pos in [0.0, 1.0].

    Encoding the frame by hand (rather than via a pyvesc message class) makes
    this independent of whether the installed pyvesc auto-scales its fields.

    Args:
        pos: Servo position in [0.0, 1.0]; 0.5 = centre.

    Returns:
        Fully framed, CRC'd bytes ready to write to the serial port.
    """
    pos = max(0.0, min(1.0, pos))
    val = int(round(pos * 1000))  # 0..1000, always non-negative
    payload = bytes([COMM_SET_SERVO_POS, (val >> 8) & 0xFF, val & 0xFF])
    crc = crc16(payload)
    return (
        bytes([_FRAME_START_SHORT, len(payload)])
        + payload
        + bytes([(crc >> 8) & 0xFF, crc & 0xFF, _FRAME_END])
    )


# ── Response ─────────────────────────────────────────────────────────────────
def parse_imu_payload(payload: bytes) -> dict:
    """Parse a de-framed COMM_GET_IMU_DATA payload into a value dict.

    Expects the payload AFTER framing/CRC have been stripped, i.e. starting
    with the command id:  [65][mask_hi][mask_lo][float ...]

    Each set bit in the mask corresponds to exactly one big-endian float32,
    in IMU_FIELD_ORDER. (VESC encodes with buffer_append_float32_auto, which
    is bit-identical to big-endian IEEE-754, so '>f' decodes it directly.)

    Args:
        payload: Raw payload bytes (id + mask + floats).

    Returns:
        Dict mapping present field names to floats. Missing fields are absent;
        use .get(name, 0.0) downstream if you need a default. Returns {} if the
        payload is too short or not an IMU frame.
    """
    if len(payload) < 3 or payload[0] != COMM_GET_IMU_DATA:
        return {}

    mask = (payload[1] << 8) | payload[2]
    body = payload[3:]

    out = {}
    offset = 0
    for bit, name in enumerate(IMU_FIELD_ORDER):
        if mask & (1 << bit):
            if offset + 4 > len(body):
                break  # truncated frame — stop gracefully
            out[name] = struct.unpack_from(">f", body, offset)[0]
            offset += 4
    return out


def to_imu_dict(payload: bytes, accel_in_ms2: bool = False,
                gyro_in_rad: bool = False) -> dict:
    """Convenience: parse and assemble the structured dict threadRead publishes.

    Args:
        payload: De-framed payload starting with id byte 65.
        accel_in_ms2: If True, convert accelerometer from g to m/s^2.
        gyro_in_rad:  If True, convert gyroscope from deg/s to rad/s.

    Returns:
        Dict with keys roll/pitch/yaw, accel[3], gyro[3], mag[3], quat[4].
        Any field the firmware did not send defaults to 0.0.
    """
    v = parse_imu_payload(payload)
    g = lambda k: float(v.get(k, 0.0))

    acc_scale = G_TO_MS2 if accel_in_ms2 else 1.0
    gyro_scale = DEG_TO_RAD if gyro_in_rad else 1.0

    return {
        "roll": g("roll"),
        "pitch": g("pitch"),
        "yaw": g("yaw"),
        "accel": [g("acc_x") * acc_scale, g("acc_y") * acc_scale, g("acc_z") * acc_scale],
        "gyro": [g("gyro_x") * gyro_scale, g("gyro_y") * gyro_scale, g("gyro_z") * gyro_scale],
        "mag": [g("mag_x"), g("mag_y"), g("mag_z")],
        "quat": [g("q0"), g("q1"), g("q2"), g("q3")],
    }


# ── Motor telemetry (COMM_GET_VALUES, id 4) ───────────────────────────────────
COMM_GET_VALUES = 4


def build_values_request() -> bytes:
    """Build a framed COMM_GET_VALUES request (payload is just the id byte)."""
    payload = bytes([COMM_GET_VALUES])
    crc = crc16(payload)
    return (
        bytes([_FRAME_START_SHORT, len(payload)])
        + payload
        + bytes([(crc >> 8) & 0xFF, crc & 0xFF, _FRAME_END])
    )


def parse_get_values(payload: bytes):
    """Parse a de-framed COMM_GET_VALUES response by fixed offsets.

    Works on VESC FW 5.x / 6.x / 7.x: the firmware only appends new fields at
    the end, so the leading fields below are at stable positions.  This avoids
    depending on a pyvesc GetValues struct that may not match the firmware.

    Layout after the id byte (offset 0 = id = 4):
        [1:3]   temp_fet            int16  /10
        [3:5]   temp_motor          int16  /10
        [5:9]   avg_motor_current   int32  /100
        [9:13]  avg_input_current   int32  /100
        [13:17] avg_id              int32  /100   (skipped)
        [17:21] avg_iq              int32  /100   (skipped)
        [21:23] duty_now            int16  /1000
        [23:27] rpm (electrical)    int32  /1
        [27:29] v_in                int16  /10
        [29:45] amp/watt hours      int32 ×4      (skipped)
        [45:49] tachometer          int32
        [49:53] tachometer_abs      int32
        [53]    fault code          uint8

    Args:
        payload: Raw payload bytes starting with id byte 4 (framing/CRC stripped).

    Returns:
        Telemetry dict, or None if the payload is not a GetValues frame.
    """
    if len(payload) < 1 or payload[0] != COMM_GET_VALUES:
        return None

    def i16(o):
        return struct.unpack_from(">h", payload, o)[0] if o + 2 <= len(payload) else 0

    def i32(o):
        return struct.unpack_from(">i", payload, o)[0] if o + 4 <= len(payload) else 0

    return {
        "temp_fet":       i16(1) / 10.0,
        "temp_motor":     i16(3) / 10.0,
        "current_motor":  i32(5) / 100.0,
        "current_in":     i32(9) / 100.0,
        "duty_cycle":     i16(21) / 1000.0,
        "rpm":            float(i32(23)),
        "voltage":        i16(27) / 10.0,
        "amp_hours":      i32(29) / 10000.0,
        "tachometer":     i32(45),
        "tachometer_abs": i32(49),
        "mc_fault_code":  payload[53] if len(payload) > 53 else 0,
    }


# ── Motor RPM (COMM_SET_RPM, id 8) ─────────────────────────────────────────────
COMM_SET_RPM = 8


def build_rpm_command(erpm: int) -> bytes:
    """Build a framed COMM_SET_RPM command (closed-loop electrical RPM).

    Args:
        erpm: Signed electrical RPM (negative = reverse).

    Returns:
        Fully framed, CRC'd bytes. Hand-framed to avoid pyvesc field-scaling
        ambiguity (eRPM is sent as a raw signed int32).
    """
    payload = bytes([COMM_SET_RPM]) + struct.pack(">i", int(erpm))
    crc = crc16(payload)
    return (
        bytes([_FRAME_START_SHORT, len(payload)])
        + payload
        + bytes([(crc >> 8) & 0xFF, crc & 0xFF, _FRAME_END])
    )
