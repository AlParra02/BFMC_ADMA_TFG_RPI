# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import re
import serial
import serial.tools.list_ports
import threading
from threading import Lock

from src.templates.workerprocess import WorkerProcess
from src.hardware.serialhandler.threads.filehandler import FileHandler
from src.hardware.serialhandler.threads.threadRead import threadRead
from src.hardware.serialhandler.threads.threadWrite import threadWrite
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.statemachine.systemMode import SystemMode
from src.utils.messages.allMessages import StateChange, SerialConnectionState


# ──────────────────────────────────────────────────────────────────────────────
# Serial port detection pattern
# ──────────────────────────────────────────────────────────────────────────────
# The original Nucleo always appeared on ttyACM*.
# A VESC connected over USB also appears on ttyACM*, but one connected via a
# USB-to-UART adapter (e.g. CP2102 / CH340) appears on ttyUSB*.
# Both patterns are covered here; the first matching port wins.
_VESC_PORT_PATTERN = re.compile(r"/dev/tty(ACM|USB)\d+")


class processSerialHandler(WorkerProcess):
    """Manages the serial connection between the Raspberry Pi and the VESC.

    Replaces the previous Nucleo-oriented version.  All structural logic
    (reconnection, thread lifecycle, state-machine integration) is unchanged.
    The following VESC-specific shared state attributes are added so that
    threadWrite and threadRead can exchange data without an extra queue
    round-trip:

    Attributes added for VESC:
        lastSteerAngle (float): Last steering angle in degrees commanded by
            threadWrite._send_servo().  Read by threadRead to publish
            CurrentSteer without a dedicated echo packet (the VESC does not
            report servo position in GetValues).

        batteryEnabled (bool): When True, threadRead publishes BatteryLvl
            messages derived from v_in.  Toggled by threadWrite in response
            to ToggleBatteryLvl queue messages.

        instantEnabled (bool): When True, threadRead publishes
            InstantConsumption messages derived from current_in.  Toggled by
            threadWrite in response to ToggleInstant queue messages.

        resourceMonitorEnabled (bool): When True, threadRead may publish
            ResourceMonitor messages.  Toggled by threadWrite in response to
            ToggleResourceMonitor queue messages.  Note: the VESC does not
            supply heap/stack data; this flag is kept for interface
            compatibility and may be used to gate CPU/memory stats collected
            locally on the Raspberry Pi instead.

    Args:
        queueList (dict): Dictionary of multiprocessing.Queue objects keyed
            by priority string ("Critical", "Warning", "General", "Config").
        logging (logging.Logger): Logger instance for debugging output.
        ready_event (threading.Event, optional): Signalled when the process
            is ready.  Defaults to None.
        dashboard_ready (threading.Event, optional): Waited on before
            sending the initial SerialConnectionState notification.
        debugging (bool): Enables verbose per-message logging.
        example (bool): Activates the built-in steering/speed sweep demo.
    """

    # ===================================== INIT =========================================

    def __init__(self, queueList, logging, ready_event=None, dashboard_ready=None,
                 debugging=False, example=False):

        logFile = "temp/serial_history.log"

        self.logger          = logging
        self.queuesList      = queueList
        self.debugging       = debugging
        self.example         = example
        self.dashboard_ready = dashboard_ready

        # ── Serial connection state ───────────────────────────────────────────
        self.serialCon       = None
        self.serialConnected = False
        self.serialDevice    = None
        self.serialLock      = Lock()
        self.reconnecting    = False

        # ── VESC shared state (read by threadRead, written by threadWrite) ────
        # Threading note: these are plain Python bools/floats.  They are only
        # written by one thread (threadWrite) and read by another (threadRead).
        # CPython's GIL makes individual reads/writes of simple types atomic,
        # so no additional lock is needed here.

        self.lastSteerAngle          = 0.0    # degrees; updated by threadWrite._send_servo()
        self.batteryEnabled          = True   # gated by ToggleBatteryLvl
        self.instantEnabled          = True   # gated by ToggleInstant
        self.resourceMonitorEnabled  = False  # gated by ToggleResourceMonitor

        # ── Supporting objects ────────────────────────────────────────────────
        self._init_subscribers()
        self._init_senders()

        self.historyFile = FileHandler(logFile)

        super(processSerialHandler, self).__init__(self.queuesList, ready_event)

    # ─────────────────────────────── subscribers / senders ──────────────────────

    def _init_subscribers(self):
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, "lastOnly", True)
        self.serialConnectionStateSubscriber = messageHandlerSubscriber(
            self.queuesList, SerialConnectionState, "lastOnly", True)

    def _init_senders(self):
        self.serialConnectedSender = messageHandlerSender(
            self.queuesList, SerialConnectionState)

    # ─────────────────────────────── serial helpers ─────────────────────────────

    def _safe_close_serial(self):
        """Safely close the serial connection with proper error handling."""
        if self.serialCon and hasattr(self.serialCon, "is_open") and self.serialCon.is_open:
            try:
                self.serialCon.close()
            except (OSError, serial.SerialException) as e:
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;93mWARNING\033[0m - Error closing serial connection: {e}"
                )
            except Exception as e:
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;91mERROR\033[0m - Unexpected error closing serial: {e}"
                )

    def _find_vesc_port(self):
        """Return the first serial port that matches the VESC port pattern.

        Checks ttyACM* (USB CDC, most common for VESC over USB) and ttyUSB*
        (USB-to-UART adapters).  Returns None if no matching port is found.

        Returns:
            str | None: Device path such as '/dev/ttyACM0', or None.
        """
        for port in serial.tools.list_ports.comports():
            if _VESC_PORT_PATTERN.match(port.device):
                return port.device
        return None

    def _try_serial_connection(self):
        """Attempt to open a serial connection to the VESC.

        Uses 115200 baud (match the baud rate configured in VESC Tool under
        App → UART).  The input/output buffers are flushed after opening so
        no stale bytes from a previous session are processed.

        Sets self.serialConnected = True on success, False on failure.
        """
        with self.serialLock:
            try:
                self._safe_close_serial()

                device = self._find_vesc_port()
                if device is None:
                    raise FileNotFoundError("No VESC serial port found")

                self.serialDevice = device
                self.serialCon    = serial.Serial(
                    self.serialDevice,
                    baudrate=115200,   # must match VESC Tool App → UART baud rate
                    timeout=0.1,
                )
                self.serialCon.reset_input_buffer()
                self.serialCon.reset_output_buffer()
                self.serialConnected = True
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;92mINFO\033[0m - Connected to VESC on "
                    f"\033[94m{self.serialDevice}\033[0m"
                )

            except (serial.SerialException, FileNotFoundError) as e:
                print(
                    f"\033[1;97m[ Serial Handler ] :\033[0m "
                    f"\033[1;93mWARNING\033[0m - Could not connect to VESC: {e}"
                )
                self._safe_close_serial()
                self.serialCon       = None
                self.serialConnected = False

    def _try_reconnect(self):
        """Attempt to reconnect; reschedules itself every second until success."""
        if self.reconnecting:
            return  # another attempt already in progress

        self.reconnecting = True
        self._try_serial_connection()

        if self.serialConnected:
            self.serialConnectedSender.send(True)
            self._reset_thread_error_states()
            self.resume_threads()
            self.reconnecting = False
        else:
            self.reconnecting = False
            threading.Timer(1, self._try_reconnect).start()

    def _reset_thread_error_states(self):
        """Clear per-thread error timestamps after a successful reconnection."""
        if self.threads:
            for thread in self.threads:
                if hasattr(thread, "last_error_time"):
                    thread.last_error_time = None

    def _wait_for_dashboard_and_notify(self):
        """Block until dashboard_ready is set, then send initial connection state."""
        self.dashboard_ready.wait()
        if self.dashboard_ready.is_set():
            self.serialConnectedSender.send(self.serialConnected)

    def _handle_serial_disconnection(self):
        """Pause threads and schedule reconnection after a disconnection event."""
        with self.serialLock:
            if self.reconnecting or not self.serialConnected:
                return  # already handling it

            print(
                f"\033[1;97m[ Serial Handler ] :\033[0m "
                f"\033[1;93mWARNING\033[0m - VESC serial device disconnected"
            )

            self.serialConnected = False
            self._safe_close_serial()
            self.serialCon = None

            if self.threads:
                self.pause_threads()

        threading.Timer(1, self._try_reconnect).start()

    # ===================================== RUN ==========================================

    def run(self):
        """Connect to the VESC, notify the dashboard, and start the threads."""
        self._try_serial_connection()

        if not self.serialConnected:
            print(
                f"\033[1;97m[ Serial Handler ] :\033[0m "
                f"\033[1;93mWARNING\033[0m - No VESC found at startup — "
                f"will keep retrying"
            )
            threading.Timer(1, self._try_reconnect).start()

        if self.dashboard_ready is not None:
            if self.dashboard_ready.is_set():
                self.serialConnectedSender.send(self.serialConnected)
            else:
                threading.Thread(
                    target=self._wait_for_dashboard_and_notify, daemon=True
                ).start()

        super(processSerialHandler, self).run()
        self.historyFile.close()

    # ===================================== PROCESS WORK =================================

    def process_work(self):
        """Check for serial disconnection events published by the threads."""
        msg = self.serialConnectionStateSubscriber.receive()
        if msg is False:
            self._handle_serial_disconnection()

    # ================================ STATE CHANGE HANDLER ==============================

    def state_change_handler(self):
        """React to system state-machine transitions."""
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            mode_dict = SystemMode[message].value["serial_handler"]["process"]
            if mode_dict["enabled"] is True:
                if self.serialConnected:
                    self.resume_threads()
            elif mode_dict["enabled"] is False:
                self.pause_threads()

    # ===================================== STOP ==========================================

    def stop(self):
        """Zero the VESC outputs, close serial, and stop all threads."""
        # Best-effort safe stop: send duty=0 directly before threads terminate.
        # This guards against the car rolling away if the process is killed
        # while the engine is enabled.
        with self.serialLock:
            if self.serialCon and self.serialConnected and self.serialCon.is_open:
                try:
                    import pyvesc
                    self.serialCon.write(pyvesc.encode(pyvesc.SetDutyCycle(0.0)))
                    self.serialCon.write(pyvesc.encode(pyvesc.SetCurrentBrake(0.0)))
                except Exception:
                    pass  # best-effort only — don't block shutdown

            if self.serialCon:
                try:
                    self.serialCon.close()
                except Exception as e:
                    print(
                        f"\033[1;97m[ Serial Handler ] :\033[0m "
                        f"\033[1;93mWARNING\033[0m - Error closing serial port: {e}"
                    )

        super(processSerialHandler, self).stop()

    # ===================================== INIT THREADS =================================

    def _init_threads(self):
        """Initialise threadRead and threadWrite with the shared process reference."""
        readTh  = threadRead(
            self, self.historyFile, self.queuesList, self.logger, self.debugging
        )
        writeTh = threadWrite(
            self, self.historyFile, self.queuesList, self.logger,
            self.debugging, self.example
        )
        self.threads.extend([readTh, writeTh])

        if not self.serialConnected:
            self.pause_threads()


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processSerialHandler.py

if __name__ == "__main__":
    from multiprocessing import Queue
    import logging
    import time

    queueList = {
        "Critical": Queue(),
        "Warning":  Queue(),
        "General":  Queue(),
        "Config":   Queue(),
    }
    logger  = logging.getLogger()
    process = processSerialHandler(queueList, logger, example=True)
    process.daemon = True
    process.start()
    time.sleep(4)
    process.stop()
