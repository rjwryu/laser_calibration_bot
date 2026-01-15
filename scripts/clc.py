#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Performs closed-loop motor control (position, velocity, current). Sends command
output to stdout. Press Ctrl-C (interrupt) to stop.
"""

import argparse, signal, can
from rmd_controller import RMDController


is_running = False


def handle_interrupt(sig, frame):
    global is_running
    is_running = False


def main(command: str, value: float, interface: str, motor_id: int):
    global is_running
    is_running = True

    signal.signal(signal.SIGINT, handle_interrupt)

    with can.Bus(channel=interface, interface="socketcan") as bus:
        motor = RMDController(motor_id, bus)

        while is_running:
            if command == "p":
                motor.set_position(value, 100)
            elif command == "v":
                motor.set_speed(value)
            elif command == "c":
                motor.set_current(value)
            else:
                motor.stop_motor()
                raise NotImplementedError("Unsupported motor command.")
            feedback = motor.get_motor_feedback()
            if feedback:
                print(f"position: {feedback.position}°, velocity: {feedback.speed}°/s, current: {feedback.current:+10.5}A")

        motor.stop_motor()


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
        "-c", "--command",
        required=True,
        choices=["p", "v", "c"],
        help="Motor command. Required."
    )
    parser.add_argument(
        "-l", "--value",
        type=float,
        required=True,
        help="Motor command value, in degrees/dps/amperes (depending on command). Required."
    )

    args = parser.parse_args()
    main(args.command, args.value, args.interface, args.motor)
