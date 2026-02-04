#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Live plotting of motor feedback.
"""

from collections import deque

from PySide6.QtCore import QThread
from PySide6.QtWidgets import QVBoxLayout, QWidget
import pyqtgraph as pg


class LiveMotorPlotWindow(QWidget):
    def __init__(self, data_source, sample_window: int):
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
        self._worker = data_source

        # Move worker to the new thread
        self._worker.moveToThread(self._thread)

        # Connect signals
        self._thread.started.connect(self._worker.run)
        self._worker.motor_feedback_signal.connect(self.update_plot)
        self._worker.error_signal.connect(self.close)

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
        if not self._thread.wait(3000):
            print("Warning: Timeout waiting for thread to stop")
        event.accept()
