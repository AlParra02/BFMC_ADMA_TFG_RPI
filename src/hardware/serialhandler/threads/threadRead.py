# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.

import math
import os
import struct
import threading
from datetime import datetime, timedelta

import pyvesc

from src.hardware.serialhandler.threads.vesc_imu import GetImuData

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.allMessages import (
    AliveSignal,
    BatteryLvl,
    CalibPWMData,
    CalibRunDone,
    CurrentSpeed,
    CurrentSteer,
    EnableButton,
    ImuAck,
    ImuData,
    InstantConsumption,
    ResourceMonitor,
    SerialConnectionState,
    ShutDownSignal,
    SteeringLimits,
    VescImuData,
    VescTelemetry,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender


# ──────────────────────────────────────────────────────────────────────────────
# Tuning constants
# ──────────────────────────────────────────────────────────────────────────────

# Motor pole pairs × gear ratio.  Used to convert electrical RPM → wheel RPM.
# Example: 7 pole-pairs, 4.00:1 gearbox  →  MOTOR_POLE_PAIRS = 7, GEAR_RATIO = 4.0
MOTOR_POLE_PAIRS = 7
GEAR_RATIO       = 4.0

# Wheel circumference in metres.  Used to convert wheel RPM → m/s.
# Example: 60 mm radius  →  2 × π × 0.060 ≈ 0.377 m
WHEEL_CIRCUMFERENCE_M = 0.377

# LiPo cell count and nominal full/empty voltages.  Used to convert VESC
# v_in (volts) to the integer percentage expected by BatteryLvl consumers.
# 3S: 12.6 V full / 9.0 V empty.  Adjust to your pack.
BATTERY_CELLS         = 3
CELL_VOLTAGE_FULL     = 4.2    # volts per cell at 100 %
CELL_VOLTAGE_EMPTY    = 3.0    # volts per cell at 0 %
BATTERY_VOLTAGE_FULL  = BATTERY_CELLS * CELL_VOLTAGE_FULL   # 12.6 V
BATTERY_VOLTAGE_EMPTY = BATTERY_CELLS * CELL_VOLTAGE_EMPTY  # 9.0 V

# VESC binary protocol constants
COMM_GET_IMU_DATA = 65     # packet ID for IMU response
VESC_FRAME_START  = 0x02   # short-frame start byte
VESC_FRAME_END    = 0x03   # frame stop byte


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _erpm_to_ms(erpm: float) -> float:
    """Convert electrical RPM reported by VESC to vehicle speed in m/s.

    Args:
        erpm: Electrical RPM from GetValues response (can be negative).

    Returns:
        Speed in m/s (positive = forward, negative = reverse).
    """
    mechanical_rpm = erpm / MOTOR_POLE_PAIRS
    wheel_rpm      = mechanical_rpm / GEAR_RATIO
    speed_ms       = (wheel_rpm / 60.0) * WHEEL_CIRCUMFERENCE_M
    return speed_ms


def _voltage_to_battery_pct(v_in: float) -> int:
    """Convert VESC input voltage to a battery percentage integer.

    Args:
        v_in: Input voltage in volts from GetValues response.

    Returns:
        Battery percentage clamped to [0, 100].
    """
    pct = (v_in - BATTERY_VOLTAGE_EMPTY) / (
        BATTERY_VOLTAGE_FULL - BATTERY_VOLTAGE_EMPTY
    ) * 100.0
    return max(0, min(100, round(pct)))


def _crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE used by the VESC binary protocol.

    Args:
        data: Payload bytes (excluding framing bytes and CRC itself).

    Returns:
        16-bit CRC integer.
    """
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
    return crc & 0xFFFF


def _extract_short_frames(buf: bytes):
    """Scan a byte buffer and yield complete short VESC frames.

    A short frame has the format:
        [0x02] [payload_len:1] [...payload...] [crc_hi] [crc_lo] [0x03]

    Args:
        buf: Raw byte buffer accumulated from the serial port.

    Yields:
        Tuple (payload: bytes, end_index: int) for each valid complete frame
        found.  ``end_index`` is the index of the byte *after* 0x03, so the
        caller can slice the buffer to remove consumed bytes.
    """
    i = 0
    while i < len(buf):
        if buf[i] != VESC_FRAME_START:
            i += 1
            continue

        # Need at least: start(1) + len(1) + payload(>=1) + crc(2) + stop(1) = 6
        if i + 5 > len(buf):
            break

        payload_len = buf[i + 1]
        frame_end   = i + 2 + payload_len + 2 + 1   # points past 0x03

        if frame_end > len(buf):
            break   # incomplete frame — wait for more bytes

        stop_byte = buf[frame_end - 1]
        if stop_byte != VESC_FRAME_END:
            i += 1
            continue  # malformed — skip this start byte

        payload  = buf[i + 2 : i + 2 + payload_len]
        crc_recv = (buf[i + 2 + payload_len] << 8) | buf[i + 2 + payload_len + 1]
        crc_calc = _crc16(payload)

        if crc_recv != crc_calc:
            i += 1
            continue  # CRC mismatch — skip

        yield payload, frame_end
        i = frame_end


# ──────────────────────────────────────────────────────────────────────────────
# threadRead
# ──────────────────────────────────────────────────────────────────────────────

class threadRead(ThreadWithStop):
    """Reads binary frames from the VESC over serial and publishes decoded
    data to the BFMC message queues.

    Replaces the Nucleo ASCII parser entirely.  The public interface
    (constructor signature, start/stop, thread_work) is identical to the
    original so that processSerialHandler requires no structural changes.

    Handles two VESC response types:
      - GetValues  (COMM_GET_VALUES, id=4)  — motor telemetry, published
        at whatever rate threadWrite requests it.
      - COMM_GET_IMU_DATA (id=65)  — IMU telemetry, published at the rate
        threadWrite polls it (default 50 Hz).

    It also re-publishes legacy messages (ImuData, BatteryLvl, CurrentSpeed,
    etc.) so that existing dashboard and state-machine subscribers continue
    to work without modification.

    Args:
        process (processSerialHandler): Parent process owning serialCon
            and serialLock.  Also carries the toggle flags
            batteryEnabled, instantEnabled, and resourceMonitorEnabled
            set by threadWrite.
        logFile (FileHandler): Log file for received frame hex strings.
        queueList (dict): Shared message queue dictionary.
        logger (logging.Logger): Logger instance.
        debugger (bool): Enables per-message debug logging when True.
    """

    # ===================================== INIT =====================================

    def __init__(self, process, logFile, queueList, logger, debugger=False):
        super(threadRead, self).__init__(pause=0.01)

        self.process    = process
        self.logFile    = logFile
        self.queuesList = queueList
        self.logger     = logger
        self.debugger   = debugger

        # Raw byte accumulation buffer (replaces the original ASCII string buffer)
        self._buf = b""

        # Tracks whether the one-shot ImuAck has been sent
        self._imu_ack_sent = False

        # Rate-limiting for serial error reporting
        self.last_error_time = None
        self.error_cooldown  = timedelta(seconds=3)

        self._init_senders()

        # Start the 1 Hz EnableButton heartbeat (identical to original)
        self._queue_sending()

    # ─────────────────────────────── senders ────────────────────────────────────

    def _init_senders(self):
        """Create messageHandlerSender instances for every published message."""
        self.enableButtonSender          = messageHandlerSender(self.queuesList, EnableButton)
        self.batteryLvlSender            = messageHandlerSender(self.queuesList, BatteryLvl)
        self.instantConsumptionSender    = messageHandlerSender(self.queuesList, InstantConsumption)
        self.imuDataSender               = messageHandlerSender(self.queuesList, ImuData)
        self.imuAckSender                = messageHandlerSender(self.queuesList, ImuAck)
        self.resourceMonitorSender       = messageHandlerSender(self.queuesList, ResourceMonitor)
        self.currentSpeedSender          = messageHandlerSender(self.queuesList, CurrentSpeed)
        self.currentSteerSender          = messageHandlerSender(self.queuesList, CurrentSteer)
        self.warningSender               = messageHandlerSender(self.queuesList, ShutDownSignal)
        self.serialConnectionStateSender = messageHandlerSender(self.queuesList, SerialConnectionState)
        self.calibPWMDataSender          = messageHandlerSender(self.queuesList, CalibPWMData)
        self.calibRunDoneSender          = messageHandlerSender(self.queuesList, CalibRunDone)
        self.steeringLimitsSender        = messageHandlerSender(self.queuesList, SteeringLimits)
        self.aliveSignalSender           = messageHandlerSender(self.queuesList, AliveSignal)
        # New VESC-specific senders
        self.vescImuDataSender           = messageHandlerSender(self.queuesList, VescImuData)
        self.vescTelemetrySender         = messageHandlerSender(self.queuesList, VescTelemetry)

    # ─────────────────────────────── heartbeat ──────────────────────────────────

    def _queue_sending(self):
        """Publish EnableButton=True every second (identical to original)."""
        self.enableButtonSender.send(True)
        threading.Timer(1, self._queue_sending).start()

    # ───────────────────────────────── main loop ────────────────────────────────

    def thread_work(self):
        """Called every pause seconds (10 ms) by ThreadWithStop.

        Reads all available bytes from the serial port, appends them to the
        internal buffer, then scans for complete VESC frames and dispatches
        each one to the appropriate handler.
        """
        try:
            with self.process.serialLock:
                con = self.process.serialCon
                if con is None or not self.process.serialConnected or not con.is_open:
                    return

                waiting = con.in_waiting
                if waiting > 0:
                    try:
                        chunk = con.read(waiting)
                        self._buf += chunk
                        if self.logFile:
                            self.logFile.write(chunk.hex())
                    except Exception as e:
                        if self._should_send_error():
                            self.serialConnectionStateSender.send(False)
                            print(
                                f"\033[1;97m[ Serial Handler ] :\033[0m "
                                f"\033[1;91mERROR\033[0m - Reading from serial ({e})"
                            )
                        return

            # ── Frame extraction (outside the lock — no serial access needed) ──
            consumed_up_to = 0
            for payload, end_idx in _extract_short_frames(self._buf):
                try:
                    self._dispatch(payload)
                except Exception as e:
                    print(
                        f"\033[1;97m[ Serial Handler ] :\033[0m "
                        f"\033[1;91mERROR\033[0m - Dispatching VESC frame "
                        f"(cmd={payload[0] if payload else '?'}): {e}"
                    )
                consumed_up_to = end_idx

            # Discard all fully consumed bytes from the front of the buffer.
            # Incomplete trailing bytes are kept for the next iteration.
            if consumed_up_to:
                self._buf = self._buf[consumed_up_to:]

            # Safety: prevent unbounded growth if only garbage arrives
            if len(self._buf) > 4096:
                if self.debugger:
                    self.logger.warning(
                        "[threadRead] Buffer overflow — clearing stale bytes"
                    )
                self._buf = b""

        except Exception as e:
            if self._should_send_error():
                self.serialConnectionStateSender.send(False)
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;91mERROR\033[0m - Thread work method ({e})"
                )

    # ─────────────────────────────── dispatcher ─────────────────────────────────

    def _dispatch(self, payload: bytes):
        """Route a validated VESC payload to the correct handler.

        Uses the first byte of the payload (the VESC command ID) to select
        the handler.  Unknown command IDs are silently ignored so that future
        VESC firmware additions do not crash the thread.

        Args:
            payload: Raw payload bytes (after framing and CRC have been
                stripped and verified by _extract_short_frames).
        """
        if not payload:
            return

        cmd_id = payload[0]

        if cmd_id == 4:
            # COMM_GET_VALUES — motor telemetry
            self._handle_get_values(payload)

        elif cmd_id == COMM_GET_IMU_DATA:
            # COMM_GET_IMU_DATA — inertial measurement unit
            self._handle_imu_data(payload)

        else:
            if self.debugger:
                self.logger.info(
                    f"[threadRead] Unhandled VESC cmd_id={cmd_id} "
                    f"({len(payload)} bytes)"
                )

    # ──────────────────────────── GetValues handler ──────────────────────────────

    def _handle_get_values(self, payload: bytes):
        """Parse a COMM_GET_VALUES response and publish telemetry messages.

        Publishes:
          - VescTelemetry     — full raw telemetry dict (new)
          - CurrentSpeed      — m/s converted from eRPM
          - CurrentSteer      — last servo angle echoed from process attribute
          - BatteryLvl        — percentage converted from v_in
          - InstantConsumption— current_in in amps

        The pyvesc decode() function handles the binary unpacking.

        Args:
            payload: Raw payload bytes starting with cmd byte 0x04.
        """
        try:
            # Reconstruct a minimal framed packet so pyvesc.decode() accepts it
            crc   = _crc16(payload)
            frame = (
                bytes([VESC_FRAME_START, len(payload)])
                + payload
                + bytes([crc >> 8, crc & 0xFF, VESC_FRAME_END])
            )
            msg, _ = pyvesc.decode(frame)

            if msg is None:
                if self.debugger:
                    self.logger.warning(
                        "[threadRead] pyvesc.decode returned None for GetValues"
                    )
                return

            # ── VescTelemetry (new, full dict) ────────────────────────────────
            telemetry = {
                "rpm":            float(getattr(msg, "rpm",            0)),
                "duty_cycle":     float(getattr(msg, "duty_now",       0)),
                "voltage":        float(getattr(msg, "v_in",           0)),
                "current_motor":  float(getattr(msg, "current_motor",  0)),
                "current_in":     float(getattr(msg, "current_in",     0)),
                "tachometer":     int(getattr(msg,   "tachometer",     0)),
                "tachometer_abs": int(getattr(msg,   "tachometer_abs", 0)),
                "temp_fet":       float(getattr(msg, "temp_fet",       0)),
                "mc_fault_code":  int(getattr(msg,   "mc_fault_code",  0)
                                     if not isinstance(
                                         getattr(msg, "mc_fault_code", 0), bytes
                                     ) else 0),
            }
            self.vescTelemetrySender.send(telemetry)

            if self.debugger:
                self.logger.info(f"[threadRead] VescTelemetry: {telemetry}")

            # ── CurrentSpeed (legacy, in m/s) ─────────────────────────────────
            speed_ms = _erpm_to_ms(telemetry["rpm"])
            self.currentSpeedSender.send(float(speed_ms))

            # ── CurrentSteer (legacy — echoed from last commanded servo angle)
            # threadWrite writes process.lastSteerAngle = float(degrees) every
            # time it calls _send_servo(), so both threads share state without
            # an extra queue round-trip.
            last_steer = getattr(self.process, "lastSteerAngle", 0.0)
            self.currentSteerSender.send(float(last_steer))

            # ── BatteryLvl (legacy, gated by toggle flag) ─────────────────────
            if getattr(self.process, "batteryEnabled", True):
                pct = _voltage_to_battery_pct(telemetry["voltage"])
                self.batteryLvlSender.send(pct)

            # ── InstantConsumption (legacy, gated by toggle flag) ─────────────
            if getattr(self.process, "instantEnabled", True):
                self.instantConsumptionSender.send(telemetry["current_in"])

            # ── Alive confirmation ─────────────────────────────────────────────
            # GetValues is now also used as the alive ping (threadWrite has no
            # dedicated firmware-version getter available in this pyvesc build).
            # Any successfully decoded GetValues response confirms the link is up.
            self.aliveSignalSender.send(True)
            self.serialConnectionStateSender.send(True)

        except Exception as e:
            print(
                f"\033[1;97m[ Serial Handler ] :\033[0m "
                f"\033[1;91mERROR\033[0m - Parsing GetValues ({e})"
            )

    # ─────────────────────────────── IMU handler ────────────────────────────────

    def _handle_imu_data(self, payload: bytes):
        """Parse a COMM_GET_IMU_DATA response and publish IMU messages.

        Decoding is delegated to pyvesc via the GetImuData message class
        registered in vesc_imu.py (VESCMessage metaclass). This replaces the
        previous hand-rolled struct.unpack_from approach.

        IMPORTANT: the field order/units in vesc_imu.GetImuData.fields must
        match your firmware's actual COMM_GET_IMU_DATA response layout.  If
        pyvesc.decode() raises or returns None here, the field list in
        vesc_imu.py needs adjusting against your firmware's commands.c /
        datatypes.h (check via VESC Tool -> Firmware).

        Publishes:
          - VescImuData  — full dict with rpy / accel / gyro / mag / quat
          - ImuData      — legacy string (roll/pitch/yaw + accel) for the
                           dashboard — no dashboard changes needed
          - ImuAck       — sent once after the first successful parse,
                           replicating the Nucleo handshake the state machine
                           expects

        Args:
            payload: Raw payload bytes starting with cmd byte 0x41 (65),
                as extracted by _extract_short_frames (framing/CRC already
                validated).
        """
        try:
            # Reconstruct a minimal framed packet so pyvesc.decode() accepts it
            crc   = _crc16(payload)
            frame = (
                bytes([VESC_FRAME_START, len(payload)])
                + payload
                + bytes([crc >> 8, crc & 0xFF, VESC_FRAME_END])
            )
            msg, _ = pyvesc.decode(frame)

            if msg is None or not isinstance(msg, GetImuData):
                if self.debugger:
                    self.logger.warning(
                        f"[threadRead] pyvesc.decode returned unexpected "
                        f"result for IMU frame: {msg!r}"
                    )
                return

            roll, pitch, yaw = msg.roll, msg.pitch, msg.yaw
            ax, ay, az       = msg.acc_x, msg.acc_y, msg.acc_z
            gx, gy, gz       = msg.gyro_x, msg.gyro_y, msg.gyro_z
            mx, my, mz       = msg.mag_x, msg.mag_y, msg.mag_z
            q0, q1, q2, q3   = msg.q0, msg.q1, msg.q2, msg.q3

            # ── VescImuData (new, full dict) ──────────────────────────────────
            imu_dict = {
                "roll":  round(roll,  4),
                "pitch": round(pitch, 4),
                "yaw":   round(yaw,   4),
                "accel": [round(ax, 4), round(ay, 4), round(az, 4)],
                "gyro":  [round(gx, 4), round(gy, 4), round(gz, 4)],
                "mag":   [round(mx, 4), round(my, 4), round(mz, 4)],
                "quat":  [round(q0, 6), round(q1, 6), round(q2, 6), round(q3, 6)],
            }
            self.vescImuDataSender.send(imu_dict)

            # ── ImuData (legacy — dashboard expects this exact dict shape) ────
            legacy_imu = {
                "roll":   str(round(roll,  2)),
                "pitch":  str(round(pitch, 2)),
                "yaw":    str(round(yaw,   2)),
                "accelx": str(round(ax,    4)),
                "accely": str(round(ay,    4)),
                "accelz": str(round(az,    4)),
            }
            self.imuDataSender.send(str(legacy_imu))

            # ── ImuAck (sent once — mirrors Nucleo one-shot handshake) ────────
            if not self._imu_ack_sent:
                self.imuAckSender.send("ack")
                self._imu_ack_sent = True

            if self.debugger:
                self.logger.info(
                    f"[threadRead] IMU  rpy=({roll:.1f}, {pitch:.1f}, {yaw:.1f}) deg  "
                    f"accel=({ax:.3f}, {ay:.3f}, {az:.3f}) m/s2  "
                    f"gyro=({gx:.3f}, {gy:.3f}, {gz:.3f}) deg/s"
                )

        except Exception as e:
            print(
                f"\033[1;97m[ Serial Handler ] :\033[0m "
                f"\033[1;91mERROR\033[0m - Parsing IMU data ({e})"
            )

    # ──────────────────────────── error rate limiting ────────────────────────────

    def _should_send_error(self) -> bool:
        """Return True if the error cooldown period has elapsed.

        Prevents flooding the gateway queue with repeated serial-error
        messages within a short window.
        """
        now = datetime.now()
        if (
            self.last_error_time is None
            or (now - self.last_error_time) >= self.error_cooldown
        ):
            self.last_error_time = now
            return True
        return False