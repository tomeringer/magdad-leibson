import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Networking Config
UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

# Data storage
x_data = []
left_y = []
right_y = []

fig, ax = plt.subplots()
line_l, = ax.plot([], [], label='Left Encoder')
line_r, = ax.plot([], [], label='Right Encoder')
ax.legend()
ax.set_xlabel("Time (Samples)")
ax.set_ylabel("Revolutions")


def update(frame):
    try:
        # Pull the latest packet from the buffer
        data, addr = sock.recvfrom(1024)
        l_val, r_val = map(float, data.decode().split(','))

        left_y.append(l_val)
        right_y.append(r_val)
        x_data.append(len(left_y))

        # Keep only last 100 points for performance
        if len(x_data) > 100:
            x_data.pop(0)
            left_y.pop(0)
            right_y.pop(0)

        line_l.set_data(x_data, left_y)
        line_r.set_data(x_data, right_y)

        ax.relim()
        ax.autoscale_view()
    except BlockingIOError:
        pass  # No data yet
    return line_l, line_r


ani = FuncAnimation(fig, update, interval=20, cache_frame_data=False)
plt.show()