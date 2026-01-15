#!/usr/bin/env python3
# Author: Su Jing Long, Brian
desc = """\
Reads mappings from angular position to current requirements, then polls the
motor for its position and applies a current from the mappings.
"""

import sys, argparse, csv, signal, can
from typing import TextIO
import numpy as np
from rmd_controller import RMDController


is_running = False


def setup_interpolation_source(input_csv: TextIO):
    """
    Read mappings from file.

    Returns a tuple (keys, values) where keys is a list of angles and values is
    a list of corresponding currents.
    """
    reader = csv.reader(input_csv)
    next(reader, None)  # skip header
    keys = []
    values = []
    for row in reader:
        keys.append(float(row[0]))
        values.append(float(row[1]))
    combined = sorted(zip(keys, values))
    keys, values = zip(*combined)
    return keys, values


def handle_interrupt(sig, frame):
    global is_running
    is_running = False


def main(can_channel: str, motor_id: int, input_file: TextIO) -> None:
    global is_running
    is_running = True

    signal.signal(signal.SIGINT, handle_interrupt)
    keys, values = setup_interpolation_source(input_file)

    with can.Bus(channel=can_channel, interface="socketcan") as bus:
        motor = RMDController(motor_id, bus)

        while is_running:
            feedback = motor.get_motor_feedback()
            if feedback is None:
                continue
            angle = feedback.position % 360
            current_setpoint = np.interp(angle, keys, values)

            feedback = motor.set_current(current_setpoint)
            if feedback is None:
                continue
            print(f"Shaft angle: {feedback.position}°, Current setpoint: {current_setpoint:+10.5}A")

        motor.shutdown_motor()


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
        "-f", "--file",
        type=argparse.FileType("r"),
        default=sys.stdin,
        help="Path to the input file (defaults to stdin)"
    )

    args = parser.parse_args()
    main(args.interface, args.motor, args.file)
