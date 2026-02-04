#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
My python library for RMD motor control.
"""

import time, struct
from dataclasses import dataclass
import can

def starts_with(start_data: bytes | list[int]):
    return lambda data: data.startswith(bytes(start_data))

def create_timeout_msg(action_name: str="motor response"):
    return f"Warning: Timeout waiting for {action_name}"

@dataclass
class MotorFeedback:
    """
    Motor feedback from several commands.

    Contains:
        - Temperature (°C) as int
        - Current (A) as float
        - Speed (°/s) as int
        - Position (°) as int
    """

    temperature: int
    current: float
    speed: int
    position: int

    def __init__(self, data: bytes):
        t, i, v, p = struct.unpack("<xbhhh", data)
        self.temperature = t * 1
        self.current = i * 0.01
        self.speed = v * 1
        self.position = p * 1

class RMDController:
    """
    MyActuator RMD motor controller. Python wrapper of RMD CAN bus protocol.
    """

    def __init__(self, motor_id: int, bus: can.BusABC):
        """
        Initialise the motor with motor ID and CAN bus handle.
        """

        self.motor_id = motor_id
        self.bus = bus

    def send(self, data: bytes | list[int]):
        """
        Sends a series of bytes as a CAN message.

        :param data: Data to send
        """

        msg = can.Message(arbitration_id=0x140 + self.motor_id, data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            print("CAN send error:", e)

    def receive(self, timeout: float=1.0):
        """
        Receive a single CAN message from the bus.

        :param timeout: Wait timeout in seconds (default 1.0)
        :return: can.Message object or None if timeout expires
        """

        try:
            msg = self.bus.recv(timeout=timeout)
            return msg
        except can.CanOperationError as e:
            print("CAN receive error:", e)
            return None

    def wait_for_msg(self, msg_filter, timeout=0.5, can_id: int | None=None, timeout_msg: str=create_timeout_msg()) -> bytes | None:
        """
        Wait for a CAN message to be received that matches the given filter.

        :param msg_filter: A function that takes a byte array and returns a boolean
        :param timeout: Wait timeout in seconds (default 0.5)
        :param can_id: The CAN ID to wait for (default is 0x240 + the motor ID)
        :return: The received message data as bytes, or None if the timeout expires
        """

        if can_id is None:
            can_id = 0x240 + self.motor_id

        end_time = time.time() + timeout
        while True:
            now = time.time()
            if now > end_time:
                print(timeout_msg)
                return None

            msg = self.receive(timeout=end_time - now)
            if msg and msg.arbitration_id == can_id and msg_filter(msg.data):
                return bytes(msg.data)

    def stop_motor(self, spam_interval: float=0.1, spam_max: int=5) -> bool:
        """
        Stop the motor.

        :param spam_interval: Time interval in seconds to spam stop commands if no reply was received (default 0.1s)
        :param spam_max: Max number of times to spam stop commands (default 5)
        :return: True on success, False on error
        """

        reply = None
        for _ in range(spam_max):
            self.send(struct.pack("<Bxxxxxxx", 0x81))
            reply = self.wait_for_msg(starts_with([0x81]), timeout=spam_interval, timeout_msg=create_timeout_msg("stop_motor"))
            if reply:
                return True

        return False

    def shutdown_motor(self, spam_interval: float=0.1, spam_max: int=5) -> bool:
        """
        Shut down the motor.

        Be careful of using this when a load is attached, since the motor will immediately de-energise and become limp.

        :param spam_interval: Time interval in seconds to spam shutdown commands if no reply was received (default 0.1s)
        :param spam_max: Max number of times to spam shutdown commands (default 5)
        :return: True on success, False on error
        """

        reply = None
        for _ in range(spam_max):
            self.send(struct.pack("<Bxxxxxxx", 0x80))
            reply = self.wait_for_msg(starts_with([0x80]), timeout=spam_interval, timeout_msg=create_timeout_msg("shutdown_motor"))
            if reply:
                return True

        return False


    def set_speed(self, speed: float) -> MotorFeedback | None:
        """
        Set the motor speed to a given value (in deg/s).

        :param speed: The desired motor speed in deg/s (float)
        :return: Motor feedback as received from the command, or None on timeout
        """

        dps = int(speed * 100)  # speed in dps
        self.send(struct.pack("<Bxxxi", 0xA2, dps))

        reply = self.wait_for_msg(starts_with([0xA2]), timeout_msg=create_timeout_msg("set_speed"))
        if reply is None:
            return None

        return MotorFeedback(reply)

    def set_current(self, current: float) -> MotorFeedback | None:
        """
        Set the motor current to a given value (in A).

        :param current: The desired motor current in A (float)
        :return: Motor feedback as received from the command, or None on timeout
        """

        i = int(current / 0.01)
        self.send(struct.pack("<Bxxxhxx", 0xA1, i))

        reply = self.wait_for_msg(starts_with([0xA1]), timeout_msg=create_timeout_msg("set_current"))
        if reply is None:
            return None

        return MotorFeedback(reply)

    def set_position(self, position: float, max_speed: float = 0) -> MotorFeedback | None:
        """
        Set the motor absolute position to a given value (in degrees).

        :param position: The desired motor position in degrees (float)
        :param max_speed: The maximum speed at which to move to the desired position (in dps, float)
        :return: Motor feedback as received from the command, or None on timeout
        """

        p = int(position / 0.01)
        mv = int(max_speed / 1)
        self.send(struct.pack("<Bxhi", 0xA4, mv, p))

        reply = self.wait_for_msg(starts_with([0xA4]), timeout_msg=create_timeout_msg("set_position"))
        if reply is None:
            return None

        return MotorFeedback(reply)

    def get_position(self) -> float | None:
        """
        Get the current position of the motor in degrees. Precision is up to 0.01°.

        :return: The current position of the motor in degrees, or None on timeout
        """

        self.send(struct.pack("<Bxxxxxxx", 0x92))

        reply = self.wait_for_msg(starts_with([0x92]), timeout_msg=create_timeout_msg("get_position"))
        if reply is None:
            return None

        return 0.01 * struct.unpack("<xxxxi", reply)[0]

    def get_motor_feedback(self) -> MotorFeedback | None:
        """
        Get the current motor feedback.

        Position is accurate to 1 degree, velocity is accurate to 1 dps, current is accurate to 0.01 A.

        :return: A tuple of (position, velocity, current) or None if no valid response is received.
        """

        self.send(struct.pack("<Bxxxxxxx", 0x9C))

        reply = self.wait_for_msg(starts_with([0x9C]), timeout_msg=create_timeout_msg("get_motor_feedback"))
        if reply is None:
            return None

        return MotorFeedback(reply)

    def set_pid_params(self, params: list[float]) -> None:
        """
        Sets the P/I/D parameters for current, velocity and position loop.

        The parameters are not saved after power off/reset of the motor.

        :param params: List of 7 floats [Kp_cur, Ki_cur, Kp_vel, Ki_vel, Kp_pos, Ki_pos, Kd_pos]
        """

        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        for i in range(len(params)):
            self.send(struct.pack("<BBxxf", 0x31, function_code[i], float(params[i])))

            reply = self.wait_for_msg(starts_with([0x31, function_code[i]]), timeout_msg=create_timeout_msg("set_pid_params"))
            if reply is None:
                print(f"Warning: No acknowledgment received for setting PID param {i}.")

    def save_pid_params(self, params: list[float]) -> None:
        """
        Saves the P/I/D parameters for current, velocity and position loop to ROM.

        :param params: List of 7 floats [Kp_cur, Ki_cur, Kp_vel, Ki_vel, Kp_pos, Ki_pos, Kd_pos]
        """

        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        for i in range(len(params)):
            self.send(struct.pack("<BBxxf", 0x32, function_code[i], float(params[i])))

            reply = self.wait_for_msg(starts_with([0x32, function_code[i]]), timeout_msg=create_timeout_msg("save_pid_params"))
            if reply is None:
                print(f"Warning: No acknowledgment received for saving PID param {i}.")

    def get_pid_params(self) -> list[float] | None:
        """
        Get the current PID parameters.

        :return: List of [Kp_cur, Ki_cur, Kp_vel, Ki_vel, Kp_pos, Ki_pos, Kd_pos], or None if an
        invalid response is received.
        """

        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        params = []
        for i in range(len(function_code)):
            self.send(struct.pack("<BBxxxxxx", 0x30, function_code[i]))

            reply = self.wait_for_msg(starts_with([0x30, function_code[i]]), timeout_msg=create_timeout_msg("get_pid_params"))
            if reply is None:
                print(f"Warning: No response received for PID param {i}")
                return None

            p = struct.unpack("<xxxxf", reply)[0]
            params.append(p)

        return params

    def set_max_acc(self, acc: list[float]) -> None:
        """
        Set the maximum accelerations for the speed and position loop.

        :param acc: List of [acc_pos, dec_pos, acc_vel, dec_vel] in deg/s²
        """

        for i in range(4):
            self.send(struct.pack("<BBxxi", 0x43, i, acc[i]))

            reply = self.wait_for_msg(starts_with([0x43]), timeout_msg=create_timeout_msg("set_max_acc"))
            if reply is None:
                print(f"Warning: No response received for setting max accel param {i}.")

    def get_max_acc(self) -> list[float] | None:
        """
        Get the acceleration limits of the motor in deg/s².

        :return: A list of [acc_pos, dec_pos, acc_vel, dec_vel] in deg/s², or
        None if no valid response received.
        """

        acc = []
        for i in range(4):
            self.send(struct.pack("<BBxxxxxx", 0x42, i))

            reply = self.wait_for_msg(starts_with([0x42, i]), timeout_msg=create_timeout_msg("get_max_acc"))

            if reply is None:
                print(f"Warning: No response received for acceleration limit {i}.")
                return None

            a = struct.unpack("<xxxxi", reply)[0]
            acc.append(a)

        return acc

    def reset_zero_pos(self) -> None:
        """
        Sets the current motor position as the zero position.

        This is saved even after a poweroff.
        """

        self.send(struct.pack("<Bxxxxxxx", 0x64))

        reply = self.wait_for_msg(starts_with([0x64]), timeout_msg=create_timeout_msg("reset_zero_pos"))

        if reply is None:
            print("Warning: No response received for encoder position re-zero.")
            return

        self.reset()
        time.sleep(1)

    def reset(self) -> None:
        """
        Soft-reset the motor.

        Note: The motor does not reply to this command.
        """

        self.send(struct.pack("<Bxxxxxxx", 0x76))
