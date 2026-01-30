#!/usr/bin/env python3
# Author: Su Jing Long, Brian
desc = """\
Incrementally move the motor to positions from 0° to 360° and record the
feedback current. Then save this to an output file.
"""

import time, csv, argparse, math
from collections import deque
from statistics import fmean
from typing import TextIO

import can
import numpy as np

from rmd_controller import RMDController


def main(can_channel: str, motor_id: int, output_file: TextIO) -> None:
    MAX_ANGLE_ERROR = 0.1
    MAX_SPEED_ERROR = 0.1
    CUTOFF_DUR = 8
    MIN_START_TIME = 4
    QUEUE_LEN = 32
    NUM_TRIALS = 24

    with can.Bus(channel=can_channel, interface="socketcan") as bus:
        motor = RMDController(motor_id, bus)

        angle = motor.get_position()
        if angle is None:
            return

        print(f"Initial angle: {angle}°")

        print(f"Resetting angle... ", end="")
        motor.reset_zero_pos()
        print("Done")

        writer = csv.writer(output_file)
        writer.writerow(["timestamp", "shaft_angle", "shaft_speed", "current"])
        recent_positions = deque([], maxlen=QUEUE_LEN)
        recent_speeds = deque([], maxlen=QUEUE_LEN)

        for i in np.linspace(0, 360, num=NUM_TRIALS, endpoint=False):
            recent_positions.clear()

            print(f"Trying angle {i} deg...")
            feedback = motor.set_position(i, 100)
            if feedback is None:
                print("Error: No response from motor.")
                return

            start_time = time.time()
            end_time = start_time + CUTOFF_DUR

            # on every communication tick
            while True:
                now = time.time()

                # enforce cutoff as first priority
                if now > end_time:
                    break

                # wait for at least min start time before recording
                if now - start_time < MIN_START_TIME:
                    continue

                feedback = motor.get_motor_feedback()
                if feedback is None:
                    continue

                angle_error = feedback.position - fmean(recent_positions) if len(recent_positions) > 0 else math.inf
                recent_positions.append(feedback.position)
                speed_error = feedback.speed - fmean(recent_speeds) if len(recent_speeds) > 0 else math.inf
                recent_speeds.append(feedback.speed)

                # wait until position and speed readings stabilise
                if abs(angle_error) > MAX_ANGLE_ERROR or abs(speed_error) > MAX_SPEED_ERROR:
                    continue

                print(f"  Current: {feedback.current:+.6f} A")
                writer.writerow([now, feedback.position, feedback.speed, feedback.current])

        motor.shutdown_motor()
        time.sleep(1)
    print("Test complete")


if __name__ == "__main__":
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
        "-o", "--output",
        required=True,
        type=argparse.FileType("w"),
        help="Path to the output file"
    )

    args = parser.parse_args()
    main(args.interface, args.motor, args.output)
