import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# --- CONFIGURATION ---
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
WINDOW_SIZE = 100  # Approx 5 seconds at 20Hz (0.05s interval)

# Networking Setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Deque automatically handles the "moving window" by popping old data
x_data = deque(maxlen=WINDOW_SIZE)
left_y = deque(maxlen=WINDOW_SIZE)
right_y = deque(maxlen=WINDOW_SIZE)

# Setup Plot
fig, ax = plt.subplots(figsize=(10, 5))
line_l, = ax.plot([], [], label='Left Encoder (Revs)', color='blue', linewidth=2)
line_r, = ax.plot([], [], label='Right Encoder (Revs)', color='orange', linewidth=2)

ax.set_title("Real-Time Encoder Feedback (5s Window)")
ax.set_xlabel("Samples")
ax.set_ylabel("Total Revolutions")
ax.grid(True, linestyle='--', alpha=0.6)
ax.legend(loc='upper left')


def update(frame):
    try:
        # Pull any available packets from the buffer
        while True:
            data, addr = sock.recvfrom(1024)
            l_val, r_val = map(float, data.decode().split(','))

            left_y.append(l_val)
            right_y.append(r_val)

            # Create a simple incrementing x-axis
            if not x_data:
                x_data.append(0)
            else:
                x_data.append(x_data[-1] + 1)

    except BlockingIOError:
        # No more data in the buffer for this frame
        pass

    if x_data:
        line_l.set_data(list(x_data), list(left_y))
        line_r.set_data(list(x_data), list(right_y))

        # Adjust the view to follow the window
        ax.set_xlim(min(x_data), max(x_data))

        # Dynamically adjust Y limits with a little padding
        all_y = list(left_y) + list(right_y)
        ax.set_ylim(min(all_y) - 0.1, max(all_y) + 0.1)

    return line_l, line_r


# interval=50 matches the Pi's 0.05s sleep (20fps)
ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()