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

import threading
import time
from datetime import datetime, timedelta

import pyvesc

from src.utils.messages.allMessages import (
    Brake,
    Control,
    ControlCalib,
    IsAlive,
    Klem,
    RequestSteerLimits,
    SerialConnectionState,
    SpeedMotor,
    SteerMotor,
    ToggleBatteryLvl,
    ToggleImuData,
    ToggleInstant,
    ToggleResourceMonitor,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.templates.threadwithstop import ThreadWithStop


# ──────────────────────────────────────────────────────────────────────────────
# Tuning constants – adjust to match your specific car and VESC configuration
# ──────────────────────────────────────────────────────────────────────────────

# Duty-cycle range.  The dashboard/Nucleo protocol used integer percent values
# in the range [-100, 100].  VESC SetDutyCycle expects a float in [-1.0, 1.0].
DUTY_SCALE = 1.0 / 100.0          # multiply int speed value by this

# Steering servo range.  The dashboard sends integer degrees in [-25, 25].
# SetServoPosition expects a float in [0.0, 1.0] where 0.5 = straight ahead.
# Adjust STEER_HALF_RANGE to match the physical limit of your steering servo.
STEER_CENTER    = 0.5             # 0.5 = center
STEER_HALF_RANGE = 25.0           # degrees that map to ±0.5 around center

# IMU polling interval in seconds.  50 Hz is a sensible default.
IMU_POLL_INTERVAL = 0.02

# Brake current in amps sent when a brake command is received.
# The original Nucleo protocol repurposed "steerAngle" as the brake value;
# map it directly to regenerative braking current here.
BRAKE_CURRENT_SCALE = 1.0         # 1 A per unit from dashboard


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _degrees_to_servo(degrees: float) -> float:
    """Convert a steering angle in degrees to a VESC servo position [0.0, 1.0].

    Args:
        degrees: Steering angle. Negative = left, positive = right.

    Returns:
        Servo position clamped to [0.0, 1.0].
    """
    pos = STEER_CENTER + (degrees / STEER_HALF_RANGE) * 0.5
    return max(0.0, min(1.0, pos))


def _speed_to_duty(speed_int: int) -> float:
    """Convert an integer speed percentage [-100, 100] to duty cycle [-1.0, 1.0].

    Args:
        speed_int: Integer speed value from the dashboard.

    Returns:
        Duty cycle clamped to [-1.0, 1.0].
    """
    duty = speed_int * DUTY_SCALE
    return max(-1.0, min(1.0, duty))


# ──────────────────────────────────────────────────────────────────────────────
# threadWrite
# ──────────────────────────────────────────────────────────────────────────────

class threadWrite(ThreadWithStop):
    """Reads commands from the BFMC message queues and forwards them to the
    VESC over serial using the pyvesc binary protocol.

    Replaces the Nucleo ASCII protocol entirely.  The public interface
    (constructor signature, start/stop, thread_work) is identical to the
    original so that processSerialHandler requires no structural changes.

    Args:
        process (processSerialHandler): Parent process that owns the serial
            port (``process.serialCon``) and the associated lock
            (``process.serialLock``).
        logFile (FileHandler): Log file for outgoing frames.
        queues (dict): Shared message queue dictionary.
        logger (logging.Logger): Logger instance.
        debugger (bool): Enables per-message debug logging.
        example (bool): Activates the built-in steering/speed sweep demo.
    """

    # ===================================== INIT =====================================

    def __init__(self, process, logFile, queues, logger,
                 debugger=False, example=False):
        super(threadWrite, self).__init__(pause=0.001)

        self.process     = process
        self.queuesList  = queues
        self.logFile     = logFile
        self.logger      = logger
        self.debugger    = debugger
        self.exampleFlag = example

        # Engine/running state (mirrors original Klem logic)
        self.running       = False
        self.engineEnabled = False

        # IMU polling timestamp
        self._last_imu_poll = 0.0

        # Error rate-limiting
        self.last_error_time = None
        self.error_cooldown  = timedelta(seconds=3)

        # Example sweep state
        if example:
            self.i = 0.0
            self.j = -1.0
            self.s = 0.0

        self._init_subscribers()
        self._init_senders()

        if example:
            self.example()

    # ─────────────────────────────── subscribers / senders ─────────────────────

    def _init_subscribers(self):
        """Create all queue subscribers (mirrors original method exactly)."""
        self.klSubscriber = messageHandlerSubscriber(
            self.queuesList, Klem, "lastOnly", True)
        self.controlSubscriber = messageHandlerSubscriber(
            self.queuesList, Control, "lastOnly", True)
        self.steerMotorSubscriber = messageHandlerSubscriber(
            self.queuesList, SteerMotor, "lastOnly", True)
        self.speedMotorSubscriber = messageHandlerSubscriber(
            self.queuesList, SpeedMotor, "lastOnly", True)
        self.brakeSubscriber = messageHandlerSubscriber(
            self.queuesList, Brake, "lastOnly", True)
        self.instantSubscriber = messageHandlerSubscriber(
            self.queuesList, ToggleInstant, "lastOnly", True)
        self.batterySubscriber = messageHandlerSubscriber(
            self.queuesList, ToggleBatteryLvl, "lastOnly", True)
        self.resourceMonitorSubscriber = messageHandlerSubscriber(
            self.queuesList, ToggleResourceMonitor, "lastOnly", True)
        self.imuSubscriber = messageHandlerSubscriber(
            self.queuesList, ToggleImuData, "lastOnly", True)
        self.controlCalibSubscriber = messageHandlerSubscriber(
            self.queuesList, ControlCalib, "lastOnly", True)
        self.isAliveSubscriber = messageHandlerSubscriber(
            self.queuesList, IsAlive, "lastOnly", True)
        self.requestSteerLimitsSubscriber = messageHandlerSubscriber(
            self.queuesList, RequestSteerLimits, "lastOnly", True)

    def _init_senders(self):
        """Create all queue senders."""
        self.serialConnectionStateSender = messageHandlerSender(
            self.queuesList, SerialConnectionState)
        # Used only by the example sweep
        self.steerMotorSender = messageHandlerSender(
            self.queuesList, SteerMotor)
        self.speedMotorSender = messageHandlerSender(
            self.queuesList, SpeedMotor)

    # ─────────────────────────────── serial helpers ─────────────────────────────

    def _write_vesc(self, packet: bytes) -> bool:
        """Write a pre-encoded pyvesc packet to the serial port.

        Acquires the shared serial lock so threadRead can safely coexist.

        Args:
            packet: Raw bytes to write (already framed + CRC by pyvesc).

        Returns:
            True on success, False on failure.
        """
        try:
            with self.process.serialLock:
                con = self.process.serialCon
                if con and self.process.serialConnected and con.is_open:
                    con.write(packet)
                    self.logFile.write(packet.hex())
                    return True
        except Exception as e:
            if self._should_send_error():
                self.serialConnectionStateSender.send(False)
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;91mERROR\033[0m - VESC write failed ({e})"
                )
        return False

    def _send_duty(self, speed_int: int):
        """Encode and send a SetDutyCycle command.

        Args:
            speed_int: Integer speed in percent [-100, 100].
        """
        duty   = _speed_to_duty(speed_int)
        packet = pyvesc.encode(pyvesc.SetDutyCycle(duty))
        if self.debugger:
            self.logger.info(f"[threadWrite] SetDutyCycle({duty:.3f})")
        self._write_vesc(packet)

    def _send_servo(self, steer_degrees: float):
        """Encode and send a SetServoPosition command.

        Args:
            steer_degrees: Steering angle in degrees [-25, 25].
        """
        pos    = _degrees_to_servo(steer_degrees)
        packet = pyvesc.encode(pyvesc.SetServoPosition(pos))
        if self.debugger:
            self.logger.info(f"[threadWrite] SetServoPosition({pos:.3f})")
        self.process.lastSteerAngle = steer_degrees
        self._write_vesc(packet)

    def _send_brake(self, brake_value: int):
        """Encode and send a SetCurrentBrake command.

        Args:
            brake_value: Raw brake value from the dashboard.
        """
        current = abs(int(brake_value)) * BRAKE_CURRENT_SCALE
        packet  = pyvesc.encode(pyvesc.SetCurrentBrake(current))
        if self.debugger:
            self.logger.info(f"[threadWrite] SetCurrentBrake({current:.2f} A)")
        self._write_vesc(packet)

    def _send_imu_poll(self):
        """Send a COMM_GET_IMU_DATA request (id=65) with mask 0xFFFF.

        This packet is not in pyvesc and is hand-assembled using the VESC
        small-frame format:
            [0x02, payload_len, cmd_byte, mask_hi, mask_lo, crc_hi, crc_lo, 0x03]
        """
        COMM_GET_IMU_DATA = 65
        payload = bytes([COMM_GET_IMU_DATA, 0xFF, 0xFF])
        crc     = self._crc16(payload)
        frame   = (
            bytes([0x02, len(payload)])
            + payload
            + bytes([crc >> 8, crc & 0xFF, 0x03])
        )
        self._write_vesc(frame)

    @staticmethod
    def _crc16(data: bytes) -> int:
        """CRC-16/CCITT-FALSE as used by the VESC binary protocol.

        Args:
            data: Payload bytes (excluding framing and CRC).

        Returns:
            16-bit CRC integer.
        """
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else crc << 1
        return crc & 0xFFFF

    def _send_alive(self):
        """Send a keep-alive ping.

        The VESC does not have a dedicated alive packet.  The cleanest
        equivalent is requesting the firmware version — a lightweight round-
        trip that confirms the link is up and triggers a FWVersion response
        in threadRead which can be used to update the dashboard connection
        indicator.
        """
        packet = pyvesc.encode(pyvesc.GetFirmwareVersion())
        if self.debugger:
            self.logger.info("[threadWrite] GetFirmwareVersion (alive ping)")
        self._write_vesc(packet)

    # ─────────────────────────────── Klem / engine state ────────────────────────

    def _handle_klem(self, kl_value: str):
        """React to a Klem (ignition key) state change.

        KL 30  → engine on, drive enabled.
        KL 15  → engine on, drive disabled (accessories only).
        KL  0  → full shutdown, coast to zero.

        The VESC has no Klem concept.  We map it to:
          KL 30 / KL 15 → SetDutyCycle(0) to hold the motor in a safe state.
          KL  0         → SetCurrentBrake(0) to release hold + SetDutyCycle(0).

        Args:
            kl_value: String "30", "15", or "0".
        """
        if kl_value == "30":
            self.running       = True
            self.engineEnabled = True
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0.0)))
            if self.debugger:
                self.logger.info("[threadWrite] KL30 – engine enabled")

        elif kl_value == "15":
            self.running       = True
            self.engineEnabled = False
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0.0)))
            if self.debugger:
                self.logger.info("[threadWrite] KL15 – accessories only")

        elif kl_value == "0":
            self.running       = False
            self.engineEnabled = False
            # Release braking hold then zero duty
            self._write_vesc(pyvesc.encode(pyvesc.SetCurrentBrake(0.0)))
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0.0)))
            if self.debugger:
                self.logger.info("[threadWrite] KL0 – full shutdown")

    # ─────────────────────────────── toggle handlers ────────────────────────────
    # The original thread sent toggle commands to the Nucleo so it could enable/
    # disable its own telemetry streams.  With VESC the Pi controls the poll
    # rate directly, so these flags are stored locally and used to gate the
    # periodic IMU and telemetry polls that threadRead depends on.

    def _handle_toggle_imu(self, value: str):
        """Enable or disable the periodic IMU polling.

        Args:
            value: "1" / "True" to enable, anything else to disable.
        """
        enabled = str(value) in ("1", "True")
        self._imu_enabled = enabled
        if self.debugger:
            self.logger.info(f"[threadWrite] IMU polling {'ON' if enabled else 'OFF'}")

    def _handle_toggle_battery(self, value: str):
        """Enable or disable battery telemetry requests.

        Battery level is included in every GetValues response, so this flag
        controls whether threadRead publishes BatteryLvl messages.  We store
        it on the process object so threadRead can inspect it.

        Args:
            value: "1" / "True" to enable.
        """
        self.process.batteryEnabled = str(value) in ("1", "True")

    def _handle_toggle_instant(self, value: str):
        """Enable or disable instant-consumption telemetry.

        Args:
            value: "1" / "True" to enable.
        """
        self.process.instantEnabled = str(value) in ("1", "True")

    def _handle_toggle_resource_monitor(self, value: str):
        """Enable or disable resource-monitor telemetry.

        Args:
            value: "1" / "True" to enable.
        """
        self.process.resourceMonitorEnabled = str(value) in ("1", "True")

    # ───────────────────────────────── main loop ────────────────────────────────

    def thread_work(self):
        """Called every ``pause`` seconds (1 ms) by ThreadWithStop.

        Processing order mirrors the original exactly so that the gateway
        priority model is preserved:

        1. Klem (ignition) — highest priority, always processed.
        2. Alive ping.
        3. Steer-limits request.
        4. Motor commands (speed, steer, brake, control) — only when running
           and engine is enabled.
        5. Toggle commands — always processed while running.
        6. Periodic IMU poll — rate-limited independently.
        """
        try:
            # ── 1. Klem ──────────────────────────────────────────────────────
            kl_recv = self.klSubscriber.receive()
            if kl_recv is not None:
                if self.debugger:
                    self.logger.info(kl_recv)
                self._handle_klem(str(kl_recv))

            # ── 2. Alive ping ─────────────────────────────────────────────────
            alive_recv = self.isAliveSubscriber.receive()
            if alive_recv is not None:
                if self.debugger:
                    self.logger.info(alive_recv)
                self._send_alive()

            # ── 3. Steer-limits request ───────────────────────────────────────
            # The VESC does not expose servo limits over serial.  We acknowledge
            # the request so the dashboard does not hang, but threadRead will
            # need to reply with the configured limits from a local config file.
            steer_limits_recv = self.requestSteerLimitsSubscriber.receive()
            if steer_limits_recv is not None:
                if self.debugger:
                    self.logger.info(steer_limits_recv)
                # Handled by threadRead / processSerialHandler config

            # ── 4. Motor commands (only when engine is on) ────────────────────
            if self.running and self.engineEnabled:

                brake_recv = self.brakeSubscriber.receive()
                if brake_recv is not None:
                    if self.debugger:
                        self.logger.info(brake_recv)
                    self._send_brake(int(brake_recv))

                speed_recv = self.speedMotorSubscriber.receive()
                if speed_recv is not None:
                    if self.debugger:
                        self.logger.info(speed_recv)
                    self._send_duty(int(speed_recv))

                steer_recv = self.steerMotorSubscriber.receive()
                if steer_recv is not None:
                    if self.debugger:
                        self.logger.info(steer_recv)
                    self._send_servo(float(steer_recv))

                # Compound vehicle command (time-boxed speed + steer)
                control_recv = self.controlSubscriber.receive()
                if control_recv is not None:
                    if self.debugger:
                        self.logger.info(control_recv)
                    self._send_duty(int(control_recv["Speed"]))
                    self._send_servo(float(control_recv["Steer"]))
                    # "Time" field: schedule a duty-zero after the interval
                    delay = int(control_recv["Time"]) / 1000.0
                    threading.Timer(delay, lambda: self._send_duty(0)).start()

                # Calibration variant of vehicle command
                calib_recv = self.controlCalibSubscriber.receive()
                if calib_recv is not None:
                    if self.debugger:
                        self.logger.info(calib_recv)
                    self._send_duty(int(calib_recv["Speed"]))
                    self._send_servo(float(calib_recv["Steer"]))
                    delay = int(calib_recv["Time"]) / 1000.0
                    threading.Timer(delay, lambda: self._send_duty(0)).start()

            # ── 5. Toggle commands ────────────────────────────────────────────
            if self.running:
                instant_recv = self.instantSubscriber.receive()
                if instant_recv is not None:
                    if self.debugger:
                        self.logger.info(instant_recv)
                    self._handle_toggle_instant(str(instant_recv))

                battery_recv = self.batterySubscriber.receive()
                if battery_recv is not None:
                    if self.debugger:
                        self.logger.info(battery_recv)
                    self._handle_toggle_battery(str(battery_recv))

                resource_recv = self.resourceMonitorSubscriber.receive()
                if resource_recv is not None:
                    if self.debugger:
                        self.logger.info(resource_recv)
                    self._handle_toggle_resource_monitor(str(resource_recv))

                imu_recv = self.imuSubscriber.receive()
                if imu_recv is not None:
                    if self.debugger:
                        self.logger.info(imu_recv)
                    self._handle_toggle_imu(str(imu_recv))

            # ── 6. Periodic IMU poll ──────────────────────────────────────────
            if (
                self.running
                and getattr(self, "_imu_enabled", True)
                and (time.time() - self._last_imu_poll) >= IMU_POLL_INTERVAL
            ):
                self._send_imu_poll()
                self._last_imu_poll = time.time()

        except Exception as e:
            print(
                f"\033[1;97m[ Serial Handler ] :\033[0m "
                f"\033[1;91mERROR\033[0m - {e}"
            )
            self.serialConnectionStateSender.send(False)

    # ===================================== START ====================================

    def start(self):
        super(threadWrite, self).start()

    # ===================================== STOP =====================================

    def stop(self):
        """Bring the car to a safe stop then shut down the thread.

        Sends duty=0 and releases any braking hold before exiting so the VESC
        does not hold a stale current command after the process terminates.
        """
        self.exampleFlag = False
        # Safe-stop sequence: zero duty, release brake hold
        try:
            self._write_vesc(pyvesc.encode(pyvesc.SetCurrentBrake(0.0)))
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0.0)))
        except Exception:
            pass
        super(threadWrite, self).stop()

    # ================================== EXAMPLE =====================================

    def example(self):
        """Sweep steering and proportional speed to verify the VESC connection.

        Behaviour is identical to the original: steer oscillates ±21°, speed
        tracks at 1/7 of the steer angle.  Runs on a 10 ms timer so it does
        not block thread_work.
        """
        if self.exampleFlag:
            self.speedMotorSender.send(str(int(self.s)))
            self.steerMotorSender.send(str(int(self.i)))
            self.i += self.j
            if self.i >= 21.0:
                self.i  =  21.0
                self.s  =  self.i / 7.0
                self.j *= -1.0
            if self.i <= -21.0:
                self.i  = -21.0
                self.s  =  self.i / 7.0
                self.j *= -1.0
            threading.Timer(0.01, self.example).start()

    # ================================ RATE LIMITING =================================

    def _should_send_error(self) -> bool:
        """Return True if enough time has passed since the last error report.

        Prevents flooding the gateway queue with serial-error messages.
        """
        now = datetime.now()
        if (
            self.last_error_time is None
            or (now - self.last_error_time) >= self.error_cooldown
        ):
            self.last_error_time = now
            return True
        return False
