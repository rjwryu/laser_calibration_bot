#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Live plotting of motor feedback.
"""

from collections import deque
from dataclasses import dataclass
from typing import Any

from PySide6.QtCore import QThread, QObject, Signal
from PySide6.QtWidgets import QVBoxLayout, QWidget
import pyqtgraph as pg


@dataclass
class Datapoint:
    data: float
    title: str
    units: str
    colour: str


@dataclass
class Plot:
    plot: Any
    curve: Any
    buffer: deque


class PlotDataSource(QObject):
    update_signal = Signal(float, list)
    error_signal = Signal()

    def __init__(self):
        super().__init__()

    def run(self) -> None:
        pass

    def stop(self) -> None:
        pass


class LiveMotorPlotWindow(QWidget):
    def __init__(self, data_source: PlotDataSource, sample_window: int, plot_wrap: int):
        super().__init__()

        self._sample_window = sample_window
        self._plot_wrap = plot_wrap

        self.setWindowTitle("Motor Feedback")
        self._layout = QVBoxLayout(self)

        # Setup Plot
        self._win = pg.GraphicsLayoutWidget(show=True)
        self._layout.addWidget(self._win)

        self.plots = {}
        self.times = deque(maxlen=sample_window)

        # Setup Threading
        self._thread = QThread()
        self._worker = data_source
        self._worker.moveToThread(self._thread)

        # Connect signals
        self._thread.started.connect(self._worker.run)
        self._worker.update_signal.connect(self.update_plot)
        self._worker.error_signal.connect(self.close)

        self._thread.start()

    def add_plot(self, title: str, colour: str="k", units: str=""):
        num_plots = len(self.plots)
        if num_plots > 0 and num_plots % self._plot_wrap == 0:
            self._win.nextRow()

        plot = self._win.addPlot(title=title)
        curve = plot.plot(pen=colour)
        plot.setLabel("left", units)
        buffer = deque(maxlen=self._sample_window)

        self.plots[title] = Plot(plot, curve, buffer)

    def update_plot(self, timestamp: float, datapoints: list[Datapoint]):
        """This runs in the Main Thread whenever the worker sends data."""
        # Add common timestamp
        self.times.append(timestamp)

        for dp in datapoints:
            # Add plot if not already existing
            if dp.title not in self.plots:
                self.add_plot(dp.title, dp.colour, dp.units)

            # Set the data and plot
            p = self.plots[dp.title]
            p.buffer.append(dp.data)
            p.curve.setData(list(self.times), list(p.buffer))

    def closeEvent(self, event):
        """Clean up threads when window is closed."""
        self._worker.stop()
        self._thread.quit()
        if not self._thread.wait(3000):
            print("Warning: Timeout waiting for thread to stop")
        event.accept()
