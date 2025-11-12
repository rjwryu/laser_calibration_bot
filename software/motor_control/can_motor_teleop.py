import can
import time
from pynput import keyboard

# ------------------ Configuration ------------------
MOTORS = [0x141, 0x142, 0x143, 0x144]   # your motor IDs
BASE_RPM = 1000                           # speed magnitude
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# ------------------ CAN helper ------------------
def send(id, data):
    msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    bus.send(msg)

def clear_faults_and_enable():
    for m in MOTORS:
        send(m, [0x81])  # clear fault
        time.sleep(0.05)
        send(m, [0xA1])  # enable torque
        time.sleep(0.05)
    print("✅ Motors enabled")

def stop_all():
    for m in MOTORS:
        data = [0xA2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        send(m, data)
    print("🛑 All motors stopped")

def rpm_to_bytes(rpm):
    """Convert signed RPM to little-endian 16-bit bytes"""
    val = int(rpm * 10) & 0xFFFF
    low = val & 0xFF
    high = (val >> 8) & 0xFF
    return [low, high]

def set_speed(id, rpm):
    low, high = rpm_to_bytes(rpm)
    data = [0xA2, 0x00, 0x00, 0x00 , low, high, 0x00, 0x00]
    send(id, data)

# ------------------ Teleop logic ------------------
current_rpm = {m: 0 for m in MOTORS}

def move_forward():
    print("↑ Forward")
    set_speed(0x141, BASE_RPM)
    set_speed(0x142, BASE_RPM)
    set_speed(0x143, BASE_RPM)
    set_speed(0x144, BASE_RPM)

def move_backward():
    print("↓ Backward")
    set_speed(0x141, -BASE_RPM)
    set_speed(0x142, -BASE_RPM)
    set_speed(0x143, -BASE_RPM)
    set_speed(0x144, -BASE_RPM)

def turn_left():
    print("← Left")
    set_speed(0x141, -BASE_RPM)
    set_speed(0x142, BASE_RPM)
    set_speed(0x143, -BASE_RPM)
    set_speed(0x144, BASE_RPM)

def turn_right():
    print("→ Right")
    set_speed(0x141, BASE_RPM)
    set_speed(0x142, -BASE_RPM)
    set_speed(0x143, BASE_RPM)
    set_speed(0x144, -BASE_RPM)

def on_press(key):
    try:
        if key.char in ['w', 'W']:
            move_forward()
        elif key.char in ['s', 'S']:
            move_backward()
        elif key.char in ['a', 'A']:
            turn_left()
        elif key.char in ['d', 'D']:
            turn_right()
        elif key.char in ['x', 'X']:
            stop_all()
    except AttributeError:
        pass

def on_release(key):
    if key in [keyboard.Key.up, keyboard.Key.down,
               keyboard.Key.left, keyboard.Key.right]:
        stop_all()
    if key == keyboard.Key.esc:
        stop_all()
        print("Exiting teleop...")
        return False

# ------------------ Main ------------------
if __name__ == "__main__":
    try:
        clear_faults_and_enable()
        print("Use W/A/S/D or arrow keys to move, X to stop, ESC to exit.")
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except KeyboardInterrupt:
        stop_all()
