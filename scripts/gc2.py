#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Second attempt at gravity compensation, real-time gravity compensation.
"""

import argparse
from collections import deque
from contextlib import nullcontext
import sys
import time

from PySide6.QtWidgets import QApplication
import can
import numpy as np

from gravity_rls import GravityModelParams, GravityRLS
from motor_liveplot import LivePlotDataSource, LivePlotWindow
from rmd_controller import RMDController


def exp_filter(new_data: float, old_data: float, alpha: float=0.1) -> float:
    return old_data + alpha * (new_data - old_data)


class MotorFeedbackData:
    """Structure to hold historical motor feedback data."""

    def __init__(self, maxlen=300):
        self.timestamps = deque(maxlen=maxlen)
        self.positions = deque(maxlen=maxlen)
        self.velocities = deque(maxlen=maxlen)
        self.currents = deque(maxlen=maxlen)


class MgrData:
    """Structure to hold historical mgr data."""

    def __init__(self, maxlen=300):
        self.timestamps = deque(maxlen=maxlen)
        self.mgr = deque(maxlen=maxlen)


class GCMotorController(LivePlotDataSource):
    def __init__(self, can_channel: str, motor_id: int, max_speed: float, max_current: float, outfile: str):
        super().__init__()
        self._running = False
        self._outfile = outfile
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        params = GravityModelParams(k=0.5641, b=1, j=1, mgr=1, alpha=0)
        self.gc_solver = GravityRLS(params)
        self.max_speed = max_speed
        self.max_current = max_current
        self.update_threshold_speed = 0.01      # rad / s
        self.update_threshold_accel = 0.01      # rad / s^2
        self._fb_data = MotorFeedbackData()
        self._mgr_data = MgrData()
        self._start_time = time.time()

    def _create_fb_graph(self):
        self.add_plot_signal.emit((0, 0), "position", ("degrees", "s"))
        self.add_curve_signal.emit("position", "", "y", False)
        self.add_plot_signal.emit((1, 0), "velocity", ("degrees/s", "s"))
        self.add_curve_signal.emit("velocity", "", "c", False)
        self.add_plot_signal.emit((0, 1), "current", ("amperes", "s"))
        self.add_curve_signal.emit("current", "", "m", False)

    def _update_fb_graph(self):
        self.update_curve_signal.emit("position", "", self._fb_data.timestamps, self._fb_data.positions)
        self.update_curve_signal.emit("velocity", "", self._fb_data.timestamps, self._fb_data.velocities)
        self.update_curve_signal.emit("current", "", self._fb_data.timestamps, self._fb_data.currents)
        self.update_curve_signal.emit("mgr", "", self._mgr_data.timestamps, self._mgr_data.mgr)

    def _update_fb_data(self, time: float, pos: float, vel: int, cur: float):
        self._fb_data.timestamps.append(time)
        self._fb_data.positions.append(pos)
        self._fb_data.velocities.append(vel)
        self._fb_data.currents.append(cur)

    def calibrate(self) -> None:
        SETPOINT_SPEED = 100
        MIN_SPEED_DATAPOINTS = 99
        SPEED_EPS = 10
        STABLE_SPEED_WINDOW_DEG = 90
        TRAVERSION_WINDOW_DEG = 360

        self._create_fb_graph()
        self.add_plot_signal.emit((1, 1), "mgr", ("s", ""))
        self.add_curve_signal.emit("mgr", "", False)

        # Open file for writing if it exists
        ctx = open(self._outfile, "w") if self._outfile else nullcontext()
        with ctx as outfile:
            if outfile:
                outfile.write("timestamp,position,velocity,current")

            # Start moving
            self.motor.set_speed(SETPOINT_SPEED)
            start_pos = self.motor.get_position()
            if start_pos is None:
                raise RuntimeError("Error: Could not get motor position")

            speed_is_unstable = True
            self._running = True
            while self._running:
                elapsed_time = time.time() - self._start_time
                logmsg = []

                fb = self.motor.get_motor_feedback()
                if fb is None:
                    raise RuntimeError("Error: Could not get motor feedback")

                # Emergency stop
                if np.abs(fb.speed) > self.max_speed:
                    raise RuntimeError("Fatal: Speed limit exceeded, stopping motor")

                pos = self.motor.get_position()
                if pos is None:
                    raise RuntimeError("Error: Could not get motor position")

                self._update_fb_data(elapsed_time, pos, fb.speed, fb.current)
                logmsg.append(f"{pos:+10.2f}°")
                logmsg.append(f"{fb.speed:+5d}°/s")
                logmsg.append(f"{fb.current:+6.2f}A")

                # TODO: stability check may not be necessary
                # Wait until stable speed and mark start pos
                if speed_is_unstable:
                    # Stop if cannot reach stable speed in given window
                    if pos >= start_pos + STABLE_SPEED_WINDOW_DEG:
                        raise RuntimeError("Error: Could not reach stable speed")

                    # Only take stddev if there are enough samples
                    if len(self._fb_data.velocities) >= MIN_SPEED_DATAPOINTS:
                        std = np.std(self._fb_data.velocities)
                        logmsg.append(f"std: {std:10.5f}°/s")

                        # Once stddev is below threshold, speed is considered stable
                        if std < SPEED_EPS:
                            print("Info: Stable speed reached")
                            speed_is_unstable = False
                            start_pos = pos

                else:
                    # Stop when traversed a given angular distance from start pos
                    if pos >= start_pos + TRAVERSION_WINDOW_DEG:
                        print("Info: Experiment completed successfully")
                        break

                    k = self.gc_solver.get_params().k
                    mgr = k * fb.current / np.cos(pos * np.pi / 180)
                    self._mgr_data.timestamps.append(elapsed_time)
                    self._mgr_data.mgr.append(mgr)

                    logmsg.append(f"mgr: {mgr:+10.5f}")

                    if outfile:
                        outfile.write(f"{elapsed_time},{pos},{fb.speed},{fb.current}\n")

                    self.update_curve_signal.emit("mgr", "", self._mgr_data.timestamps, self._mgr_data.mgr)

                print(", ".join(logmsg))
                self._update_fb_graph()

    # def gravity_compensation(self):
    #     self._running = True
    #     past_speed_rad = 0
    #     past_accel_rad = 0
    #     prev_time = time.time() - self._start_time
    #
    #     fb = self.motor.get_motor_feedback()
    #     if fb is None:
    #         raise RuntimeError("Error: Could not get motor feedback")
    #
    #     # Process the results of sending current
    #     angle_rad = (fb.position % 360) * np.pi / 180
    #     speed_rad = fb.speed * np.pi / 180
    #     accel_rad = 0
    #
    #     while self._running:
    #         elapsed_time = time.time() - self._start_time
    #         delta_time = elapsed_time - prev_time
    #         prev_time = elapsed_time
    #
    #         # Use model to predict the current to apply
    #         current_setpoint = self.gc_solver.predict(angle_rad, speed_rad, accel_rad)
    #         current_setpoint = np.clip(current_setpoint, -self.max_current, self.max_current)
    #         fb = self.motor.set_current(current_setpoint)
    #         if fb is None:
    #             raise RuntimeError("Error: Could not apply current")
    #
    #         # Enforce speed limit with emergency stop
    #         if np.abs(fb.speed) > self.max_speed:
    #             raise RuntimeError("Fatal: Speed limit exceeded, stopping motor")
    #
    #         # Process the results of sending current
    #         angle_rad = (fb.position % 360) * np.pi / 180
    #         speed_rad = fb.speed * np.pi / 180
    #         accel_rad_raw = (speed_rad - past_speed_rad) / delta_time
    #         accel_rad = exp_filter(accel_rad_raw, past_accel_rad, 0.1)
    #         feedback_current = fb.current
    #         past_speed_rad = speed_rad
    #         past_accel_rad = accel_rad
    #
    #         # Update coefficients of gravity model through RLS, but only when moving
    #         is_updated = False
    #         if np.abs(speed_rad) > self.update_threshold_speed or np.abs(accel_rad) > self.update_threshold_accel:
    #             self.gc_solver.update(angle_rad, speed_rad, accel_rad, feedback_current)
    #             is_updated = True
    #
    #         params = self.gc_solver.params
    #         print(f"Info: {fb.position % 360:+4d}°, {fb.speed:+6d}°/s, {fb.current:+6.2f}A, updated: {is_updated:1},  b: {params.b:+10.5f}, j: {params.j:+10.5f}, mgr: {params.mgr:+10.5f}, alpha: {params.alpha:+10.5f}")
    #
    #         self.update_signal.emit(
    #             elapsed_time,
    #             [
    #                 Datapoint(fb.position, "position", "degrees", "y"),
    #                 Datapoint(fb.speed, "speed", "degrees/s", "c"),
    #                 Datapoint(fb.current, "amperes", "amperes", "m"),
    #             ],
    #         )

    def run(self):
        try:
            print("Info: Starting calibration procedure")
            self.calibrate()
            # print("Info: Starting gravity compensation")
            # self.gravity_compensation()

        except RuntimeError as e:
            print(e)

        finally:
            self.close_signal.emit()
            print("Info: Shutting down motor")
            self.motor.shutdown_motor()
            self.bus.shutdown()

    def stop(self):
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
        "--max-speed",
        type=float,
        default=360,
        help="Speed limit before emergency stop is activated. Defaults to 360dps."
    )
    parser.add_argument(
        "--max-current",
        type=float,
        default=5,
        help="Saturation current applied by the controller. Defaults to 5A."
    )
    parser.add_argument(
        "-o", "--outfile",
        default="",
        help="Output CSV file for test data. If unspecified, does not write test data to file."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    controller = GCMotorController(args.interface, args.motor, args.max_speed, args.max_current, args.outfile)
    window = LivePlotWindow(controller, args.samples)

    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
