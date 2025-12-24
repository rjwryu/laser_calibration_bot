#! /usr/bin/env python3
import can
import time
from typing import List, Tuple
import struct

class RMDController:
    def __init__(self, motor_id: int, bus: can.BusABC):
        self.motor_id = motor_id
        self.bus = bus

    def send(self, data: bytes):
        msg = can.Message(arbitration_id=0x140 + self.motor_id, data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            print("CAN send error:", e)

    def receive(self, timeout: float = 1.0):
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

    def wait_for_msg(self, msg_filter, timeout=0.5, can_id: int=None) -> bytearray | None:
        """
        Wait for a CAN message to be received that matches the given filter.

        :param msg_filter: A function that takes a byte array and returns a boolean
        :param timeout: Wait timeout in seconds (default 0.5)
        :param can_id: The CAN ID to wait for (default is 0x240 + the motor ID)
        :return: The received message data as a byte array, or None if the timeout expires
        """
        if can_id is None:
            can_id = 0x240 + self.motor_id

        end_time = time.time() + timeout
        while True:
            now = time.time()
            if now > end_time:
                print("Warning: Timeout waiting for motor reply")
                return None
            
            msg = self.receive(timeout=end_time - now)
            if msg and msg.arbitration_id == can_id and msg_filter(msg.data):
                return msg.data

    def stop_motor(self):
        now = time.time()
        while (time.time() - now) < 0.5:
            self.send([0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            msg = self.receive(timeout=0.1)
            if msg and msg.arbitration_id == 0x240 + self.motor_id and len(msg.data) >= 8 and msg.data[0] == 0x80:
                return True
        return False

    def set_speed(self, speed: float):
        """
        Set the motor speed to a given value (in deg/s).

        :param speed: The desired motor speed in deg/s (float)
        :return: True if the motor acknowledged the command, False otherwise
        """
        dps = int(speed * 100)  # speed in dps
        b = dps.to_bytes(4, byteorder='little', signed=True)
        data = [0xA2, 0x00, 0x00, 0x00, b[0], b[1], b[2], b[3]]
        self.send(data)

        reply = self.wait_for_msg(lambda data: data[0] == 0xA2)
        return reply is not None

    def set_current(self, current: float):
        """
        Set the motor current to a given value (in A).

        :param current: The desired motor current in A (float)
        :return: True if the motor acknowledged the command, False otherwise
        """
        value = int(current * 100)  # scale: 0.01 A units
        low = value & 0xFF
        high = (value >> 8) & 0xFF
        data = [0xA1, 0x00, 0x00, 0x00, low, high, 0x00, 0x00]
        self.send(data)

        now = time.time()
        while (time.time() - now) < 0.5:
            msg = self.receive(timeout=0.1)
            if msg and msg.arbitration_id == 0x240 + self.motor_id and len(msg.data) >= 8 and msg.data[0] == 0xA1:
                return True
        return False

    def set_position(self, position: float, max_speed: float = 0) -> bool:
        """
        Set the motor absolute position to a given value (in degrees).

        :param position: The desired motor position in degrees (float)
        :param max_speed: The maximum speed at which to move to the desired position (in dps, float)
        :return: True if the motor acknowledges the command, False otherwise
        """
    
        p = int(position / 0.01)
        mv = int(max_speed)
        # data = [0xA4, 0x00, s0, s1, b0, b1, b2, b3]
        data = struct.pack('<Bxhi', 0xA4, mv, p)
        self.send(data)

        reply = self.wait_for_msg(lambda data: data[0] == 0xA4)
        return reply is not None
    
    def get_position(self) -> float | None: 
        """
        Get the current position of the motor in degrees. Precision is up to 0.01°.
        
        :return: The current position of the motor in degrees or None if no valid response is received.
        """
        self.send([0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

        reply = self.wait_for_msg(lambda data: data[0] == 0x92)
        if reply is None:
            return None
        
        pos_i = struct.unpack('<xxxxi', reply)[0]
        return pos_i * 0.01
    
    def get_motor_feedback(self) -> Tuple[float, float, float]:
        """
        Get the current motor feedback: position (degrees), velocity (degrees per second), current (amperes).
        Position is accurate to 1 degree, velocity is accurate to 1 dps, current is accurate to 0.01 A.

        Sends a CAN message requesting the motor feedback and waits for a response from the motor.
        The response is expected to be sent by the motor as a CAN message with arbitration ID
        0x24X where X is the motor ID.

        If a valid response is received within 0.5 seconds, the motor feedback is returned as a tuple
        of (position, velocity, current). Otherwise, None is returned.

        :return: A tuple of (position, velocity, current) or None if no valid response is received.
        """
        self.send([0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        now = time.time()
        while (time.time() - now) < 0.5:
            msg = self.receive(timeout=0.1)
            if msg and msg.arbitration_id == 0x240 + self.motor_id and len(msg.data) >= 8 and msg.data[0] == 0x9C:
                # Unpack as signed int16 (little-endian)
                current = struct.unpack('<h', bytes([msg.data[2], msg.data[3]]))[0]
                velocity = struct.unpack('<h', bytes([msg.data[4], msg.data[5]]))[0]
                position = struct.unpack('<h', bytes([msg.data[6], msg.data[7]]))[0]
                return position * 1.0, velocity * 1.0, current * 0.01  # position in degrees, velocity in dps, current in A
        return None

    def set_pid_params(self, params: List[float]):
        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        for i in range(len(params)):
            data = struct.pack('<BBxxf', 0x31, function_code[i], float(params[i]))
            self.send(data)

            reply = self.wait_for_msg(lambda data: data[0] == 0x31 and data[1] == function_code[i])
            if reply is None:
                print(f"Warning: No acknowledgment received for setting PID param {i}.")

    def save_pid_params(self, params: List[float]) -> None:
        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        for i in range(len(params)):
            data = struct.pack('<BBxxf', 0x32, function_code[i], float(params[i]))
            self.send(data)

            reply = self.wait_for_msg(lambda data: data[0] == 0x32 and data[1] == function_code[i])
            if reply is None:
                print(f"Warning: No acknowledgment received for saving PID param {i}.")

    def get_pid_params(self) -> List[float]:
        """
        Get the current PID parameters (Kp_cur, Ki_cur, Kp_vel, Ki_vel, Kp_pos, Ki_pos, Kd_pos).

        :return: A list of [Kp_cur, Ki_cur, Kp_vel, Ki_vel, Kp_pos, Ki_pos, Kd_pos], or None if no
        valid response is received.
        """
        function_code = (0x01, 0x02, 0x04, 0x05, 0x07, 0x08, 0x09)
        params = []
        for i in range(len(function_code)):
            data = [0x30, function_code[i], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            self.send(data)

            reply = self.wait_for_msg(lambda data: data[0] == 0x30 and data[1] == function_code[i])
            if reply is None:
                print(f"Warning: No response received for PID param {i}")
                return None
            
            val = struct.unpack('<xxxxf', reply)[0]
            params.append(val)

        return params
    
    def set_max_acc(self, acc: List[float]) -> None:
        for i in range(4):
            data = struct.pack('<BBxxi', 0x43, i, acc[i])
            self.send(data)

            reply = self.wait_for_msg(lambda data: data[0] == 0x43)
            if reply is None:
                print(f"Warning: No response received for setting max accel param {i}.")
    
    def get_max_acc(self) -> List[float]:
        """
        Get the acceleration limits of the motor in deg/s².

        Sends a CAN message requesting the maximum accelerations and waits for a response from the motor.
        The response is expected to be sent by the motor as a CAN message with arbitration ID
        0x24X where X is the motor ID.

        If a valid response is received within 0.5 seconds, the maximum accelerations are returned as a list of floats
        in deg/s². Otherwise, None is returned.

        :return: A list of [acc_pos, dec_pos, acc_vel, dec_vel] in deg/s², or None if no valid response is received.
        """
        acc = []
        for i in range(4):
            self.send([0x42, i, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

            msg_filter = lambda data: data[0] == 0x42 and data[1] == i
            reply = self.wait_for_msg(msg_filter)

            if reply is not None:
                a = struct.unpack('<i', reply[4:8])[0]
                acc.append(a)
            else:
                print(f"Warning: No response received for acceleration limit {i}.")
                break
        return acc
    
    def reset_zero_pos(self) -> None:
        self.send([0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        msg_filter = lambda data: data[0] == 0x64
        reply = self.wait_for_msg(msg_filter)

        if reply is None:
            print('Warning: No response received for encoder position re-zero.')
            return

        self.send([0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        # motor does not reply to a reset command, add delay just to be safe
        time.sleep(0.5)