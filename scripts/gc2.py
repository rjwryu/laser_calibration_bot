#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Second attempt at gravity compensation, real-time gravity compensation.
"""

import argparse
from dataclasses import dataclass
import sys
import time

from PySide6.QtCore import QObject, Signal
from PySide6.QtWidgets import QApplication
import can
import numpy as np

from motor_liveplot import LiveMotorPlotWindow
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

    def update(self, pos: float, vel: float, acc: float, cur: float):
        """
        Update the recursive solver with new data.

        :param pos: Angular position in radians
        :param vel: Angular velocity in radians/second
        :param acc: Angular acceleration in radians/second²
        :param cur: Feedback current in amperes
        :return: Learned parameters, in a tuple (C1, C2, B/K, J/K)
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
        theta = theta + K * error

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


class GCMotorController(QObject):
    motor_feedback_signal = Signal(float, dict)
    estop_signal = Signal(str)

    def __init__(self, can_channel: str, motor_id: int, max_speed: float):
        super().__init__()
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        self._start_time = time.time()
        params = GravityModelParams(k=1, b=1, j=1, mgr=1, alpha=1)
        self.gc_solver = GravityRLS(params)
        self.max_speed = max_speed

    def run(self):
        self._running = True
        past_speed = 0
        prev_time = time.time() - self._start_time

        fb = self.motor.get_motor_feedback()
        if fb is None:
            print("Error: could not get motor feedback")
            return

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
            fb = self.motor.set_current(current_setpoint)
            if fb is None:
                continue

            # Enforce speed limit with emergency stop
            if fb.speed > self.max_speed:
                self.estop_signal.emit("Fatal: Speed limit exceeded, stopping motor")
                print("Fatal: Speed limit exceeded, stopping motor")
                break

            # Process the results of sending current
            angle = (fb.position % 360) * np.pi / 180
            speed = fb.speed * np.pi / 180
            accel = (fb.speed - past_speed) / delta_time
            cur = fb.current

            # Update coefficients of gravity model through RLS
            self.gc_solver.update(angle, speed, accel, cur)
            params = self.gc_solver.params
            print(f"{fb.position % 360:+10.5f}°, {fb.current:+10.5f}A, k: {params.k:+10.5f}, b: {params.b:+10.5f}, j: {params.j:+10.5f}, mgr: {params.mgr:+10.5f}, alpha: {params.alpha:+10.5f}")

            self.motor_feedback_signal.emit(
                elapsed_time,
                {
                    "position": fb.position,
                    "speed":    fb.speed,
                    "current":  fb.current,
                },
            )

    def stop(self):
        self._running = False
        self.motor.shutdown_motor()
        self.bus.shutdown()


### 1st RLS attempt
# def main(can_channel: str, motor_id: int) -> None:
#     def handle_interrupt(sig, frame):
#         global is_running
#         is_running = False
#
#     global is_running
#     is_running = True
#     gc_solver = GravityRLS()
#
#     signal.signal(signal.SIGINT, handle_interrupt)
#
#     with can.Bus(channel=can_channel, interface="socketcan") as bus:
#         motor = RMDController(motor_id, bus)
#         motor.set_current(0)
#
#         past_speed = 0
#         prev_time = time.time()
#         while is_running:
#             fb = motor.get_motor_feedback()
#             now = time.time()
#             delta_time = now - prev_time
#             prev_time = now
#             if fb is None:
#                 continue
#             angle = (fb.position % 360) * np.pi / 180
#             speed = fb.speed * np.pi / 180
#             accel = (fb.speed - past_speed) / delta_time
#
#             # Find coefficients of gravity model through RLS
#             c1, c2, b, j = gc_solver.update(angle, speed, accel, fb.current)
#             print(f"Angle: {angle:3d}°, Current: {fb.current:+10.5f}A, c1: {c1:+10.5f}, c2: {c2:+10.5f}, b/k: {b:+10.5f}, j/k: {j:+10.5f}")
#
#             # i = C1*sin(theta) + C2*cos(theta) + B/K*omega + J/K*alpha
#             current_setpoint = c1*np.sin(angle) + c2*np.cos(angle) + b*speed + j*accel
#             fb = motor.set_current(current_setpoint)
#             if fb is None:
#                 continue
#
#         motor.shutdown_motor()

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
        default=720,
        help="Speed limit before emergency stop is activated. Defaults to 720dps."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    controller = GCMotorController(args.interface, args.motor, args.max_speed)
    window = LiveMotorPlotWindow(controller, args.samples)

    controller.estop_signal.connect(app.quit)

    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
