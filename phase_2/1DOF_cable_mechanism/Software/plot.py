import can
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ---------------------------------------
# CAN setup (SocketCAN)
# ---------------------------------------
bus = can.Bus(
    interface='socketcan',
    channel='can0',        # use the interface you brought up with `ip link`
    bitrate=1000000
)

TARGET_ID = 0x241          # replace with your actual CAN ID

# ---------------------------------------
# Data buffers
# ---------------------------------------
N = 200
t_buf = deque(maxlen=N)
a_buf = deque(maxlen=N)
b_buf = deque(maxlen=N)
c_buf = deque(maxlen=N)

# ---------------------------------------
# Matplotlib setup
# ---------------------------------------
fig, ax = plt.subplots()
line_a, = ax.plot([], [], label="Value A")
line_b, = ax.plot([], [], label="Value B")
line_c, = ax.plot([], [], label="Value C")

ax.legend()
ax.set_xlabel("Time (s)")
ax.set_ylabel("Values")
ax.set_title("Live CAN Plot (10 Hz)")

# ---------------------------------------
# Decode helper
# ---------------------------------------
def decode_values(d):
    # bytes 2–3, 4–5, 6–7 as uint16 little-endian
    a = struct.unpack_from("<H", d, 2)[0]
    b = struct.unpack_from("<H", d, 4)[0]
    c = struct.unpack_from("<H", d, 6)[0]
    return a, b, c

# ---------------------------------------
# Animation update
# ---------------------------------------
def update(frame):
    msg = bus.recv(0.1)   # 100 ms timeout — fits 10 Hz

    if msg and msg.arbitration_id == TARGET_ID and len(msg.data) >= 8:
        a, b, c = decode_values(msg.data)
        now = time.time()

        t_buf.append(now)
        a_buf.append(a)
        b_buf.append(b)
        c_buf.append(c)

        line_a.set_data(t_buf, a_buf)
        line_b.set_data(t_buf, b_buf)
        line_c.set_data(t_buf, c_buf)

        ax.relim()
        ax.autoscale_view()

    return line_a, line_b, line_c

ani = FuncAnimation(fig, update, interval=100)
plt.show()
