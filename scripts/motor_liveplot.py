#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Live plotting of feedback.
"""

from PySide6.QtCore import QObject, QThread, Signal
from PySide6.QtWidgets import QMainWindow, QVBoxLayout
import pyqtgraph as pg


class LivePlotDataSource(QObject):
    close_signal = Signal()
    add_plot_signal = Signal(tuple, str, tuple)
    add_curve_signal = Signal(str, str, str, bool)
    update_curve_signal = Signal(str, str, list, list)

    def __init__(self):
        super().__init__()

    def run(self) -> None:
        pass

    def stop(self) -> None:
        pass


class LivePlotWindow(QMainWindow):
    def __init__(self, data_source: LivePlotDataSource, sample_window: int, plot_wrap: int):
        super().__init__()

        self._sample_window = sample_window
        self._plot_wrap = plot_wrap

        self.setWindowTitle("Motor Feedback")
        self._layout = QVBoxLayout(self)

        # Setup Plot
        self._win = pg.GraphicsLayoutWidget(show=True)
        self._layout.addWidget(self._win)

        self._plots = {}
        self._curves = {}

        # Setup Threading
        self._thread = QThread()
        self._worker = data_source
        self._worker.moveToThread(self._thread)

        # Connect signals
        self._thread.started.connect(self._worker.run)
        self._worker.close_signal.connect(self.close)
        self._worker.add_plot_signal.connect(self.add_plot)
        self._worker.add_curve_signal.connect(self.add_curve)
        self._worker.update_curve_signal.connect(self.update_curve)

        self._thread.start()

    def add_plot(self, grid: tuple[int, int], title: str, units: tuple[str, str]=("","")):
        plot = self._win.addPlot(row=grid[0], col=grid[1], title=title)
        plot.showGrid(x=True, y=True)

        if units[0] != "":
            plot.setLabel("bottom", units=units[0])
        if units[1] != "":
            plot.setLabel("left", units=units[1])

        self._plots[title] = plot
        self._curves[title] = {}

    def add_curve(self, plot_title: str, label: str, colour="r", is_scatter=False):
        plot = self._plots[plot_title]

        if label == "":
            label = plot_title
        else:
            plot.addLegend()

        curve = None
        if is_scatter:
            curve = plot.scatterPlot(name=label, pen=colour)
        else:
            curve = plot.plot(name=label, pen=colour)

        self._curves[plot_title][label] = curve

    def update_curve(self, plot_title: str, label: str, x, y):
        self._curves[plot_title][label or plot_title].setData(x, y)

    def closeEvent(self, event):
        """Clean up threads when window is closed."""
        self._worker.stop()
        self._thread.quit()
        if not self._thread.wait(3000):
            print("Warning: Timeout waiting for thread to stop")
        event.accept()
