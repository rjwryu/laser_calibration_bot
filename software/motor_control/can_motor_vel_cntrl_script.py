import can
import time

# === CAN Bus Setup ===
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Motor IDs
MOTORS = [0x141, 0x142, 0x143, 0x144]

def send(id, data):
    """Send one CAN frame"""
    msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    bus.send(msg)

def enable_all():
    for m in MOTORS:
        send(m, [0x81])     # Clear fault
        time.sleep(0.05)
        send(m, [0xA1])     # Enable torque
        time.sleep(0.05)
    print("Motors enabled")

def stop_all():
    for m in MOTORS:
        send(m, [0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00])  # Stop
    print("Motors stopped")

def set_speed(id, rpm):
    """Send speed command to a motor (in RPM), Converts DEC To HEX"""
    value = int(rpm * 10)   # Protocol uses rpm * 10
    low = value & 0xFF
    high = (value >> 8) & 0xFF
    data = [0xA2, 0x00, 0x00 ,0x00, low, high, 0x00, 0x00]
    send(id, data)

def demo_motion():
    enable_all()
    time.sleep(0.5)

    # Spin each motor at different speeds
    set_speed(0x141, 10000)
    set_speed(0x142, 10000)
    set_speed(0x143, 10000)
    set_speed(0x144, 10000)

    time.sleep(5)
    stop_all()

if __name__ == "__main__":
    try:
        demo_motion()
    except KeyboardInterrupt:
        stop_all()
