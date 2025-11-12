import can
import time
from pynput import keyboard

# ------------------ Configuration ------------------
MOTORS = [0x141, 0x142, 0x143, 0x144]
ANGLE_STEP = 45        # degrees per keypress
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# ------------------ Helpers ------------------
def send(id, data):
    msg = can.Message(arbitration_id=id, data=data, is_extended_id=False)
    bus.send(msg)

def enable_motors():
    for m in MOTORS:
        send(m, [0x81])   # clear fault
        time.sleep(0.05)
        send(m, [0xA1])   # enable torque
        time.sleep(0.05)
    print("✅ Motors enabled")

def deg_to_bytes(angle_deg):
    """Convert degrees to 0.01° units (signed 16-bit little-endian)."""
    val = int(angle_deg * 1) & 0xFFFF
    low = val & 0xFF
    high = (val >> 8) & 0xFF
    return [low, high]

def move_relative(id, delta_deg):
    """Increment current position by delta_deg."""
    low, high = deg_to_bytes(delta_deg)
    frame = [0xA8, 0x00, 0x00, 0x0A,0x00,0x00, low, high ]
    send(id, frame)
    print(f"Motor {hex(id)} moved {delta_deg:+.1f}°")

def stop_all():
    for m in MOTORS:
        send(m, [0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    print("🛑 Stop command sent")

# ------------------ Teleop logic ------------------
def on_press(key):
    try:
        if key.char == '1':
            move_relative(0x141, +ANGLE_STEP)
        elif key.char == 'q':
            move_relative(0x141, -ANGLE_STEP)
        elif key.char == '2':
            move_relative(0x142, +ANGLE_STEP)
        elif key.char == 'w':
            move_relative(0x142, -ANGLE_STEP)
        elif key.char == '3':
            move_relative(0x143, +ANGLE_STEP)
        elif key.char == 'e':
            move_relative(0x143, -ANGLE_STEP)
        elif key.char == '4':
            move_relative(0x144, +ANGLE_STEP)
        elif key.char == 'r':
            move_relative(0x144, -ANGLE_STEP)
        elif key.char in ['x', 'X']:
            stop_all()
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        stop_all()
        print("Exiting teleop...")
        return False

# ------------------ Main ------------------
if __name__ == "__main__":
    try:
        enable_motors()
        print("""
Incremental Angle Teleop
------------------------
Motor1: 1/+ , q/-
Motor2: 2/+ , w/-
Motor3: 3/+ , e/-
Motor4: 4/+ , r/-
X: stop   ESC: exit
""")
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except KeyboardInterrupt:
        stop_all()
