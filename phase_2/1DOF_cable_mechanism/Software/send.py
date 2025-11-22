import can
import time
import struct

# ---------------------------------------
# CAN setup (SocketCAN)
# ---------------------------------------
bus = can.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000
)

TARGET_ID = 0x141     # choose your ID

# ---------------------------------------
# Helper: pack three uint16 values into bytes 2–7
# Layout:
#   byte 0–1: unused (set to zero)
#   byte 2–3: value A (uint16 LE)
#   byte 4–5: value B (uint16 LE)
#   byte 6–7: value C (uint16 LE)
# ---------------------------------------
def build_payload(a, b, c):
    payload = bytearray(8)
    struct.pack_into("<H", payload, 2, a)
    struct.pack_into("<H", payload, 4, b)
    struct.pack_into("<H", payload, 6, c)
    return payload

# ---------------------------------------
# Send loop at 10 Hz
# ---------------------------------------
rate = 0.1  # 100 ms = 10 Hz

while True:
    # Example values — replace or dynamically update
    val_a = 100
    val_b = 200
    val_c = 300

    data = build_payload(val_a, val_b, val_c)

    payload = bytearray(8)
    struct.pack_into("<H", payload, 2, a)
    struct.pack_into("<H", payload, 4, b)
    struct.pack_into("<H", payload, 6, c)

    msg = can.Message(
        arbitration_id=TARGET_ID,
        data=data,
        is_extended_id=False
    )

    bus.send(msg)

    time.sleep(rate)
