#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Live plotting script of motor feedback.
"""

import argparse
from collections import deque
import sys
import time

from PySide6.QtCore import QObject, QThread, Signal
from PySide6.QtWidgets import QApplication, QVBoxLayout, QWidget
import can
import pyqtgraph as pg

from rmd_controller import RMDController


class ControllerWorker(QObject):
    # Define a signal that sends a float (the sensor value)
    data_received = Signal(float, dict)

    def __init__(self, can_channel: str, motor_id: int):
        super().__init__()
        self._running = True
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        self._start_time = time.time()

    def run(self):
        """The continuous loop running in the background thread."""
        while self._running:
            current_time = time.time() - self._start_time
            fb = self.motor.get_motor_feedback()
            if fb is None:
                continue

            print(f"{fb.position:+6d}°, {fb.speed:+6d}°/s, {fb.current:+10.5f}A")
            self.data_received.emit(current_time, {
                "position": fb.position,
                "speed":    fb.speed,
                "current":  fb.current,
            })
            time.sleep(0.01)  # Control sampling rate (100Hz)

    def stop(self):
        self._running = False
        self.motor.shutdown_motor()
        self.bus.shutdown()


class LivePlotWindow(QWidget):
    def __init__(self, data_source: ControllerWorker, sample_window: int, can_channel: str, motor_id: int):
        super().__init__()
        self.setWindowTitle("Motor Feedback")
        self._layout = QVBoxLayout(self)

        # Setup Plot
        self._win = pg.GraphicsLayoutWidget(show=True)
        self._layout.addWidget(self._win)

        self._sensors = ["position", "speed", "current"]
        self._sensor_data = {
            "position": ("y", "degrees"),
            "speed":    ("c", "degrees/s"),
            "current":  ("m", "amperes"),
        }
        self._plots = {}
        self._curves = {}
        self._buffers = {}
        self._times = deque(maxlen=sample_window)
        for i in range(len(self._sensors)):
            if i == 2:
                self._win.nextRow()
            sensor = self._sensors[i]
            colour, unit = self._sensor_data[sensor]
            self._plots[sensor] = self._win.addPlot(title=sensor)
            self._curves[sensor] = self._plots[sensor].plot(pen=colour)
            self._plots[sensor].setLabel("left", unit)
            self._buffers[sensor] = deque(maxlen=sample_window)

        # --- Setup Threading ---
        self._thread = QThread()
        self._worker = ControllerWorker(can_channel, motor_id)

        # Move worker to the new thread
        self._worker.moveToThread(self._thread)

        # Connect signals
        self._thread.started.connect(self._worker.run)
        self._worker.data_received.connect(self.update_plot)

        self._thread.start()

    def update_plot(self, timestamp: float, values: dict):
        """This runs in the Main Thread whenever the worker sends data."""
        self._times.append(timestamp)
        for name, value in values.items():
            self._buffers[name].append(value)
            self._curves[name].setData(list(self._times), list(self._buffers[name]))

    def closeEvent(self, event):
        """Clean up threads when window is closed."""
        self._worker.stop()
        self._thread.quit()
        self._thread.wait()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument(
        "-i", "--interface",
        default="can0",
        help="CAN interface. Defaults to can0."
    )
    parser.add_argument(
        "-m", "--motor",
        type=int,
        default=1,
        help="Motor ID. Defaults to 1."
    )
    parser.add_argument(
        "-s", "--samples",
        type=int,
        default=200,
        help="Number of recent samples to plot. Defaults to 200."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    window = LivePlotWindow(args.samples, args.interface, args.motor)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
