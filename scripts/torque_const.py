#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Procedure to find the torque constant of a motor experimentally.
"""

import argparse
from collections import deque
import sys
import time

from PySide6.QtWidgets import QApplication
import can
import numpy as np
import pandas as pd
from scipy import stats

from motor_liveplot import LiveMotorPlotWindow, PlotDataSource
from rmd_controller import RMDController


class TorqueConstExperiment(PlotDataSource):
    def __init__(self, can_channel: str, motor_id: int, max_speed: float, train_data_output: str):
        super().__init__()
        self.bus = can.Bus(channel=can_channel, interface="socketcan")
        self.motor = RMDController(motor_id, self.bus)
        self.max_speed = max_speed
        self.train_data_output = train_data_output
        self._running = False

    def _wait_for_static(self, pos_eps: float=0.1, cur_eps: float=0.1, delay: float=3) -> tuple[float, float]:
        pos_change_sample = deque(maxlen=99)
        cur_change_sample = deque(maxlen=99)
        pos = 0
        cur = 0
        prev_pos = np.inf
        prev_cur = np.inf

        end_time = time.time() + delay
        while True:
            if not self._running:
                raise RuntimeError("Info: Cancelled by user")

            if time.time() < end_time:
                time.sleep(0.1)
                continue

            pos = self.motor.get_position()
            if pos is None:
                raise RuntimeError("Error: Couldn't get position feedback while moving to position")

            fb = self.motor.get_motor_feedback()
            if fb is None:
                raise RuntimeError("Error: Couldn't get motor feedback while moving to position")
            cur = fb.current

            pos_change_sample.append(np.abs(pos - prev_pos))
            cur_change_sample.append(np.abs(cur - prev_cur))
            if np.mean(pos_change_sample) < pos_eps and np.mean(cur_change_sample) < cur_eps:
                break

            prev_pos = pos
            prev_cur = cur

        return (pos, cur)

    def _do_trials(self, max_weights_num: int, inc_weight: float, train_data: dict[str, list[float]]):
        is_reversed = False
        num_weights = 0
        while True:
            print(f"No. of weights: {num_weights}")

            pos, cur = self._wait_for_static()
            print(f"Position: {pos:+.2f}°, Current: {cur:+.2f}")

            # Record data for training
            train_data["added_mass"].append(num_weights * inc_weight)
            train_data["current"].append(cur)

            # Repeat the experiment by adding weights, then removing them
            if not is_reversed:
                if num_weights >= max_weights_num:
                    is_reversed = True
                else:
                    num_weights += int(input("Add weights, then enter how many weights were added: "))

            if is_reversed:
                if num_weights == 0:
                    break
                else:
                    num_weights -= int(input("Remove weights, then enter how many weights were removed: "))

    def _write_results(self, df: pd.DataFrame):
        print(f"Test complete. Writing data to \"{self.train_data_output}\"")
        df.to_csv(self.train_data_output)

    def _extract_data(self, df: pd.DataFrame, gravity: float, lever_arm: float) -> float:
        slope, intercept, r_value, p_value, std_err = stats.linregress(df["added_mass"], df["current"])
        print(f"Coefficient (Slope): {slope:.4f}")
        print(f"Intercept: {intercept:.4f}")
        print(f"R-squared: {r_value**2:.4f}")

        torque_constant = gravity * lever_arm / slope
        print(f"Torque constant: {torque_constant:.6f}")
        return torque_constant

    def _run_experiment(self):
        train_data = { "added_mass": [], "current": [] }

        gravity = 9.81
        lever_arm = 0.225       # length of lever arm
        inc_weight = 0.0525     # mass of each weight to be added
        max_weights_num = 10    # total number of weights to be added

        input("Point the link straight downwards and press Enter: ")

        # Get zero position
        zero_pos = self.motor.get_position()
        if zero_pos is None:
            raise RuntimeError("Error: Couldn't get initial position")
        print(f"Zero position: {zero_pos:+.2f}°")

        # Move to 90 deg and start trials
        print(f"Info: Moving to +90°")
        self.motor.set_position(zero_pos + 90)
        self._do_trials(max_weights_num, inc_weight, train_data)

        # Move from the other direction and do trials again (to compensate for friction/hysteresis)
        self._wait_for_static()
        print("Info: Moving to +180°")
        self.motor.set_position(zero_pos + 180)

        self._wait_for_static()
        print("Info: Moving to +90°")
        self.motor.set_position(zero_pos + 90)
        self._do_trials(max_weights_num, inc_weight, train_data)

        df = pd.DataFrame(data=train_data)

        # Output to CSV
        self._write_results(df)

        # Data mining to find torque constant
        self._extract_data(df, gravity, lever_arm)

    def run(self) -> None:
        self._running = True
        try:
            self._run_experiment()

        except Exception as e:
            print(e)

        finally:
            self.error_signal.emit()
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
        "--max-speed",
        type=float,
        default=360,
        help="Speed limit before emergency stop is activated. Defaults to 360dps."
    )
    parser.add_argument(
        "-o", "--output",
        type=str,
        default="output.csv",
        help="Name of data output file to create. Defaults to \"output.csv\"."
    )

    args = parser.parse_args()
    app = QApplication(sys.argv)
    data_source = TorqueConstExperiment(args.interface, args.motor, args.max_speed, args.output)
    window = LiveMotorPlotWindow(data_source, 1, 2)

    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
