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

# IMU request is built explicitly (with the required 2-byte mask) because
# pyvesc.encode_request() emits only the id byte and the firmware would then
# return no IMU fields.  See vesc_imu.py.
from src.hardware.serialhandler.threads.vesc_imu import (
    build_imu_request,
    build_rpm_command,
    build_servo_command,
    build_values_request,
)

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
# Steering note
# ──────────────────────────────────────────────────────────────────────────────
# Steering uses the VESC servo output (COMM_SET_SERVO_POS, id 33), NOT
# SetPosition (COMM_SET_POS, motor angle).  The command frame is built by hand
# in vesc_imu.build_servo_command() so it does not depend on whether this
# pyvesc build auto-scales its message fields.


# ──────────────────────────────────────────────────────────────────────────────
# Tuning constants – adjust to match your specific car and VESC configuration
# ──────────────────────────────────────────────────────────────────────────────

# Scaling note (IMPORTANT):
# This pyvesc build does NOT auto-scale SetDutyCycle / SetCurrentBrake — it
# packs the value you pass straight into an integer field. So we must pass
# PRE-SCALED INTEGERS:
#   * duty : percent[-100,100] × 1000  → [-100000, 100000]   (100000 = duty 1.0)
#   * brake: amps × 1000               (firmware current units)
# Passing a float to these throws "required argument is not an integer".
# (Steering is hand-framed in vesc_imu.build_servo_command, so it is unaffected.)
# Verify once with the hex you log in _write_vesc: a 100% duty command must
# serialise to payload "05 00 01 86 a0" (id 5, then 100000 = 0x000186A0).
DUTY_SCALE = 1000          # multiply the [-100,100] speed value by this

# Steering servo range.  The dashboard sends integer degrees in [-25, 25].
# SetServoPosition expects a float in [0.0, 1.0] where 0.5 = straight ahead.
# Adjust STEER_HALF_RANGE to match the physical limit of your steering servo.
STEER_CENTER     = 0.5            # 0.5 = center
STEER_HALF_RANGE = 25.0           # degrees that map to ±0.5 around center

# IMU polling interval in seconds.  20 Hz keeps serial load modest.
IMU_POLL_INTERVAL = 0.05

# Telemetry (GetValues) polling interval.  The VESC only answers when asked, so
# we must poll it for the dashboard feedback (speed, battery, steer echo).
# 10 Hz is plenty for the gauges.
TELEMETRY_POLL_INTERVAL = 0.1

# Brake current in amps sent when a brake command is received.
# The original Nucleo protocol repurposed "steerAngle" as the brake value;
# map it directly to regenerative braking current here.
BRAKE_CURRENT_SCALE = 1.0         # amps per unit from dashboard

# ── Closed-loop speed control (cm/s) ──────────────────────────────────────────
# SpeedMotor from the dashboard is treated as a target speed in cm/s and driven
# via the VESC's closed-loop RPM control, so the number means real cm/s rather
# than throttle %.  These drivetrain constants MUST match threadRead.py (they
# are the inverse of its eRPM→speed conversion).
MOTOR_POLE_PAIRS      = 4
GEAR_RATIO            = 11.84
WHEEL_CIRCUMFERENCE_M = 0.314
MAX_SPEED_CMS         = 60.0      # safety clamp on the commanded speed
# Below this |cm/s| we coast (duty 0) instead of commanding RPM, because
# sensorless FOC cannot hold a near-zero RPM cleanly. Low-speed behaviour is
# left to the VESC's own open-loop/sensorless tuning.
MIN_DRIVE_CMS         = 0.5

# Output refresh (keep-alive): unlike the Nucleo, the VESC does NOT hold a
# setpoint indefinitely — it stops the motor if it receives no command within
# its configured timeout (VESC Tool → App → General → timeout).  The dashboard
# sends discrete setpoints, not a continuous stream, so we re-send the last
# commanded duty + servo position at this interval to keep them applied until
# the setpoint changes or the engine is switched off.
REFRESH_INTERVAL = 0.1            # seconds (10 Hz keep-alive)


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


def _speed_to_duty(speed_int: int) -> int:
    """Convert an integer speed percentage [-100, 100] to VESC duty units.

    Returns a PRE-SCALED INTEGER in [-100000, 100000] (100000 == duty 1.0),
    because this pyvesc build packs SetDutyCycle as a raw integer with no
    internal scaling.

    Args:
        speed_int: Integer speed value from the dashboard.

    Returns:
        Integer duty value clamped to [-100000, 100000].
    """
    duty = int(speed_int) * DUTY_SCALE
    return int(max(-100000, min(100000, duty)))


def _speed_cms_to_erpm(cms: float) -> int:
    """Convert a target speed in cm/s to electrical RPM for SetRPM.

    Inverse of threadRead._erpm_to_ms, using the same drivetrain constants.

    Args:
        cms: Target speed in cm/s (signed).

    Returns:
        Electrical RPM (signed int).
    """
    m_s        = cms / 100.0
    wheel_rps  = m_s / WHEEL_CIRCUMFERENCE_M       # wheel revolutions / second
    motor_rpm  = wheel_rps * 60.0 * GEAR_RATIO
    erpm       = motor_rpm * MOTOR_POLE_PAIRS
    return int(round(erpm))


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
        super(threadWrite, self).__init__(pause=0.005)

        self.process     = process
        self.queuesList  = queues
        self.logFile     = logFile
        self.logger      = logger
        self.debugger    = debugger
        self.exampleFlag = example

        # Engine/running state (mirrors original Klem logic)
        self.running       = False
        self.engineEnabled = False

        # IMU polling timestamp / enable flag
        self._last_imu_poll = 0.0
        self._imu_enabled   = True

        # Telemetry (GetValues) polling timestamp
        self._last_telem_poll = 0.0

        # Held output setpoints, re-sent periodically to keep the VESC alive.
        self._cur_duty     = 0                        # pre-scaled duty units
        self._cur_rpm      = 0                        # electrical RPM setpoint
        self._drive_mode   = "duty"                   # "duty" or "rpm"
        self._cur_servo    = _degrees_to_servo(0.0)   # 0.5 = centre
        self._last_refresh = 0.0

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
            packet: Raw bytes to write (already framed + CRC).

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
        """Set the held duty setpoint and send it (open-loop).

        Also selects duty drive-mode so the keep-alive refreshes duty.

        Args:
            speed_int: Integer speed in percent [-100, 100].
        """
        self._cur_duty   = _speed_to_duty(speed_int)
        self._drive_mode = "duty"
        packet = pyvesc.encode(pyvesc.SetDutyCycle(self._cur_duty))
        if self.debugger:
            self.logger.info(f"[threadWrite] SetDutyCycle({self._cur_duty})")
        self._write_vesc(packet)

    def _send_rpm(self, erpm: int):
        """Set the held eRPM setpoint and send it (closed-loop speed control).

        Selects rpm drive-mode so the keep-alive refreshes RPM.

        Args:
            erpm: Electrical RPM (signed).
        """
        self._cur_rpm    = int(erpm)
        self._drive_mode = "rpm"
        packet = build_rpm_command(self._cur_rpm)
        if self.debugger:
            self.logger.info(f"[threadWrite] SetRPM({self._cur_rpm})")
        self._write_vesc(packet)

    def _send_speed_cms(self, cms: float):
        """Drive a target speed in cm/s via closed-loop RPM.

        Clamps to ±MAX_SPEED_CMS. Near-zero coasts (duty 0) rather than
        commanding RPM 0, which sensorless FOC can't hold cleanly. Low-speed
        behaviour is left to the VESC's own open-loop/sensorless tuning.

        Args:
            cms: Target speed in cm/s (signed).
        """
        cms = max(-MAX_SPEED_CMS, min(MAX_SPEED_CMS, cms))
        if abs(cms) < MIN_DRIVE_CMS:
            self._send_duty(0)                       # coast to stop
        else:
            self._send_rpm(_speed_cms_to_erpm(cms))

    def _send_servo(self, steer_degrees: float):
        """Set the held servo setpoint and send it (steering).

        Also records the commanded angle on the parent process so threadRead
        can publish CurrentSteer (the VESC does not echo servo position).

        Args:
            steer_degrees: Steering angle in degrees [-25, 25].
        """
        pos = _degrees_to_servo(steer_degrees)
        self._cur_servo = pos
        self.process.lastSteerAngle = float(steer_degrees)
        packet = build_servo_command(pos)  # hand-framed COMM_SET_SERVO_POS
        if self.debugger:
            self.logger.info(f"[threadWrite] SetServoPos({pos:.3f})")
        self._write_vesc(packet)

    def _refresh_outputs(self):
        """Re-send the last drive + servo setpoint to keep the VESC alive.

        The VESC stops the motor if it receives no command within its
        configured timeout, so held setpoints must be refreshed periodically
        (it does not latch a setpoint the way the Nucleo firmware did).
        """
        if self._drive_mode == "rpm":
            self._write_vesc(build_rpm_command(self._cur_rpm))
        else:
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(self._cur_duty)))
        self._write_vesc(build_servo_command(self._cur_servo))

    def _send_brake(self, brake_value: int):
        """Encode and send a SetCurrentBrake command.

        Passes braking current in amps; pyvesc scales it ×1000 internally.

        Args:
            brake_value: Raw brake value from the dashboard.
        """
        current_amps  = abs(int(brake_value)) * BRAKE_CURRENT_SCALE
        current_units = int(current_amps * 1000)  # amps -> firmware integer units
        self._cur_duty   = 0                       # stop driving; refresh sends 0
        self._cur_rpm    = 0
        self._drive_mode = "duty"
        packet = pyvesc.encode(pyvesc.SetCurrentBrake(current_units))
        if self.debugger:
            self.logger.info(
                f"[threadWrite] SetCurrentBrake({current_amps:.2f} A -> {current_units})"
            )
        self._write_vesc(packet)

    def _send_imu_poll(self):
        """Send a framed COMM_GET_IMU_DATA request (id=65) with mask 0xFFFF.

        build_imu_request() includes the 2-byte field mask the firmware
        requires; without it the VESC returns no IMU floats.
        """
        if self.debugger:
            self.logger.info("[threadWrite] GetImuData request sent")
        self._write_vesc(build_imu_request())

    def _send_alive(self):
        """Send a keep-alive ping.

        pyvesc does not expose a firmware-version getter in this build.
        GetValues is used instead — a lightweight telemetry request that
        confirms the link is up.  threadRead._handle_get_values() already
        processes the response and will publish AliveSignal /
        SerialConnectionState via its normal telemetry path.
        """
        packet = build_values_request()
        if self.debugger:
            self.logger.info("[threadWrite] GetValues (alive ping)")
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
            self._cur_duty     = 0                 # start stopped
            self._cur_rpm      = 0
            self._drive_mode   = "duty"
            self._last_refresh = time.time()
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0)))
            if self.debugger:
                self.logger.info("[threadWrite] KL30 – engine enabled")

        elif kl_value == "15":
            self.running       = True
            self.engineEnabled = False
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0)))
            if self.debugger:
                self.logger.info("[threadWrite] KL15 – accessories only")

        elif kl_value == "0":
            self.running       = False
            self.engineEnabled = False
            self._cur_duty     = 0
            self._cur_rpm      = 0
            self._drive_mode   = "duty"
            # Release braking hold then zero duty
            self._write_vesc(pyvesc.encode(pyvesc.SetCurrentBrake(0)))
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0)))
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
           and engine is enabled.  Held setpoints are re-sent (keep-alive) so
           the VESC keeps applying them.
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
                    # SpeedMotor arrives as cm/s × 10 (deci-cm/s); convert to cm/s.
                    self._send_speed_cms(float(speed_recv) / 10.0)

                steer_recv = self.steerMotorSubscriber.receive()
                if steer_recv is not None:
                    if self.debugger:
                        self.logger.info(steer_recv)
                    # SteerMotor arrives as degrees × 10 (decidegrees); convert.
                    self._send_servo(float(steer_recv) / 10.0)

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

                # ── Keep-alive: re-send held duty + servo so the VESC, which
                #    does not latch setpoints, keeps applying them. ───────────
                if (time.time() - self._last_refresh) >= REFRESH_INTERVAL:
                    self._refresh_outputs()
                    self._last_refresh = time.time()

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
                and self._imu_enabled
                and (time.time() - self._last_imu_poll) >= IMU_POLL_INTERVAL
            ):
                self._send_imu_poll()
                self._last_imu_poll = time.time()

            # ── 7. Periodic telemetry poll (GetValues) ────────────────────────
            # Drives the dashboard speed gauge, battery level and steer echo.
            # The VESC does not stream telemetry; it only replies when asked.
            if (
                self.running
                and (time.time() - self._last_telem_poll) >= TELEMETRY_POLL_INTERVAL
            ):
                self._write_vesc(build_values_request())
                self._last_telem_poll = time.time()

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
            self._write_vesc(pyvesc.encode(pyvesc.SetCurrentBrake(0)))
            self._write_vesc(pyvesc.encode(pyvesc.SetDutyCycle(0)))
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