#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Second attempt at gravity compensation, real-time gravity compensation.
"""

import argparse
from dataclasses import dataclass
import sys
import time

from PySide6.QtWidgets import QApplication
import can
import numpy as np

from motor_liveplot import Datapoint, LiveMotorPlotWindow, PlotDataSource
from rmd_controller import RMDController


@dataclass
class GravityModelParams:
    k: float        # torque constant
    b: float        # friction coefficient
    j: float        # moment of inertia
    mgr: float      # mass * gravitational constant * distance of CoM
    alpha: float    # angle zero offset


class GravityRLS:
    def __init__(self, params: GravityModelParams, lmbda=0.995, delta=100.0):
        self.params = params
        self.lmbda = lmbda  # Forgetting factor (0.9 to 0.999)
        self.P = np.eye(4) * delta     # Initial uncertainty matrix
        self.j_min = 1e-6
        self.b_min = 1e-6

    def update(self, pos: float, vel: float, acc: float, cur: float) -> None:
        """
        Update the recursive solver with new data.

        :param pos: Angular position in radians
        :param vel: Angular velocity in radians/second
        :param acc: Angular acceleration in radians/second²
        :param cur: Feedback current in amperes
        """

        # 1. Create the regressor vector for the current angle
        phi = np.array([
            [acc],
            [vel],
            [np.sin(pos)],
            [np.cos(pos)],
        ])

        # 2. Prediction Error
        theta = np.array([
            [self.params.j],
            [self.params.b],
            [self.params.mgr * np.cos(self.params.alpha)],
            [self.params.mgr * np.sin(self.params.alpha)],
        ])
        error = self.params.k * cur - (phi.T @ theta)[0, 0]

        # 3. Calculate Gain Vector (K)
        # K tells us how much to change parameters based on the error
        num = self.P @ phi
        den = self.lmbda + (phi.T @ self.P @ phi)
        K = num / den

        # 4. Update Estimates
        new_theta = theta + K * error
        new_theta[0, 0] = max(new_theta[0, 0], self.j_min)
        new_theta[1, 0] = max(new_theta[1, 0], self.b_min)
        theta = new_theta

        # 5. Update Covariance Matrix (P)
        self.P = (self.P - (K @ phi.T @ self.P)) / self.lmbda

        new_params = theta.flatten()
        self.params.j = new_params[0]
        self.params.b = new_params[1]
        self.params.mgr = np.hypot(new_params[2], new_params[3])
        self.params.alpha = np.arctan2(new_params[2], new_params[3])

    def predict(self, pos: float, vel: float, acc: float) -> float:
        """
        Use learned parameters to make a prediction of current.

        :param pos: Angular position in radians
        :param vel: Angular velocity in radians/second
        :param acc: Angular acceleration in radians/second²
        :return: Current prediction in amperes
        """
        return (self.params.mgr * np.cos(pos + self.params.alpha) + self.params.b * vel + self.params.j * acc) / self.params.k


class GCMotorController(PlotDataSource):
    def __init__(self, can_channel: str, motor_id: int, max_speed: float, max_current: float):
        super().__init__()
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        params = GravityModelParams(k=0.5641, b=1, j=1, mgr=1, alpha=1)
        self.gc_solver = GravityRLS(params)
        self.max_speed = max_speed
        self.max_current = max_current
        self._start_time = time.time()

    def gravity_compensation(self):
        self._running = True
        past_speed = 0
        prev_time = time.time() - self._start_time

        fb = self.motor.get_motor_feedback()
        if fb is None:
            raise RuntimeError("Error: Could not get motor feedback")

        # Process the results of sending current
        angle = (fb.position % 360) * np.pi / 180
        speed = fb.speed * np.pi / 180
        accel = 0

        while self._running:
            elapsed_time = time.time() - self._start_time
            delta_time = elapsed_time - prev_time
            prev_time = elapsed_time

            # Use model to predict the current to apply
            current_setpoint = self.gc_solver.predict(angle, speed, accel)
            current_setpoint = np.clip(current_setpoint, -self.max_current, self.max_current)
            fb = self.motor.set_current(current_setpoint)
            if fb is None:
                raise RuntimeError("Error: Could not apply current")

            # Enforce speed limit with emergency stop
            if np.abs(fb.speed) > self.max_speed:
                raise RuntimeError("Fatal: Speed limit exceeded, stopping motor")

            # Process the results of sending current
            angle = (fb.position % 360) * np.pi / 180
            speed = fb.speed * np.pi / 180
            accel = (speed - past_speed) / delta_time
            cur = fb.current
            past_speed = speed

            # Update coefficients of gravity model through RLS
            self.gc_solver.update(angle, speed, accel, cur)
            params = self.gc_solver.params
            print(f"Info: {fb.position % 360:+10.5f}°, {fb.current:+10.5f}A, k: {params.k:+10.5f}, b: {params.b:+10.5f}, j: {params.j:+10.5f}, mgr: {params.mgr:+10.5f}, alpha: {params.alpha:+10.5f}")

            self.update_signal.emit(
                elapsed_time,
                [
                    Datapoint(fb.position, "position", "degrees", "y"),
                    Datapoint(fb.speed, "speed", "degrees/s", "c"),
                    Datapoint(fb.current, "amperes", "amperes", "m"),
                ],
            )

    def run(self):
        print("Info: Starting gravity compensation")
        try:
            self.gravity_compensation()

        except RuntimeError as e:
            print(e)

        finally:
            self.error_signal.emit()
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
        "-w", "--plot-wrap",
        type=int,
        default=2,
        help="Number of plots per row. Defaults to 2."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    controller = GCMotorController(args.interface, args.motor, args.max_speed, args.max_current)
    window = LiveMotorPlotWindow(controller, args.samples, args.plot_wrap)

    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
