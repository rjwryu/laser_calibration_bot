#! /usr/bin/env python3
import can

class RMDController:
    def __init__(self, motor_id: int, bus: can.BusABC):
        self.motor_id = motor_id
        self.bus = bus

    def send(self, data: bytes):
        msg = can.Message(arbitration_id=int(self.motor_id), data=bytes(data), is_extended_id=False)
        try:
            self.bus.send(msg)
        except Exception as e:
            print("CAN send error:", e)

    def stop_motor(self):
        self.send([0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def set_speed(self, rpm: float):
        dps = int(rpm * 600)  # 1 rpm = 600 * 0.01 dps
        low = dps & 0xFF
        high = (dps >> 8) & 0xFF
        data = [0xA2, 0x00, 0x00, 0x00, low, high, 0x00, 0x00]
        self.send(data)

    def set_current(self, current: float):
        value = int(current * 100)  # scale: 0.01 A units
        low = value & 0xFF
        high = (value >> 8) & 0xFF
        data = [0xA1, 0x00, 0x00, 0x00, low, high, 0x00, 0x00]
        self.send(data)

    def set_position(self, position: int, max_speed: float = 0):
        v = int(position) & 0xFFFFFFFF
        b0 = v & 0xFF
        b1 = (v >> 8) & 0xFF
        b2 = (v >> 16) & 0xFF
        b3 = (v >> 24) & 0xFF
        # max_speed must be provided in deg/s (dps) when calling this method.
        # The helpers in this module convert from rpm or dps as needed.
        sv = int(max_speed) & 0xFFFF if max_speed else 0
        s_low = sv & 0xFF
        s_high = (sv >> 8) & 0xFF
        data = [0xA4, 0x00, s_low, s_high, b0, b1, b2, b3]
        self.send(data)