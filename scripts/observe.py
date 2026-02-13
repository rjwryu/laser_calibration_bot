#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Observe motor feedback with real-time graphs.
"""

import argparse
import sys
import time

from PySide6.QtWidgets import QApplication
import can

from motor_liveplot import Datapoint, LiveMotorPlotWindow, PlotDataSource
from rmd_controller import RMDController


class MotorObserver(PlotDataSource):
    def __init__(self, can_channel: str, motor_id: int):
        super().__init__()
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        self._start_time = time.time()

    def run(self) -> None:
        print("Info: Starting motor")
        self._running = True
        past_speed = 0
        prev_time = time.time() - self._start_time

        while self._running:
            elapsed_time = time.time() - self._start_time
            delta_time = elapsed_time - prev_time
            prev_time = elapsed_time

            fb = self.motor.get_motor_feedback()
            if fb is None:
                continue

            accel = (fb.speed - past_speed) / delta_time
            past_speed = fb.speed

            print(f"Info: {fb.position % 360:+10d}°, {fb.speed:+10.5f}°/s, {accel:+10.5f}°/s², {fb.current:+10.5f}A")

            self.update_signal.emit(
                elapsed_time,
                [
                    Datapoint(fb.position, "position", "degrees", "y"),
                    Datapoint(fb.speed, "speed", "degrees/s", "c"),
                    Datapoint(accel, "acceleration", "degrees/s²", "w"),
                    Datapoint(fb.current, "current", "amperes", "m"),
                ],
            )

        print("Info: Shutting down motor")
        self.motor.shutdown_motor()
        self.bus.shutdown()

    def stop(self) -> None:
        self._running = False


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
    parser.add_argument(
        "-w", "--plot-wrap",
        type=int,
        default=2,
        help="Number of plots per row. Defaults to 2."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    data_source = MotorObserver(args.interface, args.motor)
    window = LiveMotorPlotWindow(data_source, args.samples, args.plot_wrap)

    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
