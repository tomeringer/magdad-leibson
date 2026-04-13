import serial
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math

# --- CONFIGURATION ---
SERIAL_PORT = 'COM6'  
BAUD_RATE = 115200
NUM_FLEX = 5  
WINDOW_SIZE = 150  

# --- IMPORT THRESHOLDS AND HYSTERESIS FROM CALIBRATION ---

# THRESHOLDS = [
#     [37002.4, 45115.9, 59251.9],
#     [18688.8, 26000.0, 35000.2],
#     [19430.9, 26701.9, 40450.2],
#     [25493.6, 30308.9, 38821.2],
#     [12044.7, 14235.7, 19452.4]
# ]

# # Updated to 2D Array
# HYSTERESIS = [
#     [1500.0, 1500.0, 1500.0],
#     [1000.0, 1000.0, 1000.0],
#     [1000.0, 1000.0, 1000.0],
#     [857.6,  857.6,  857.6],
#     [700.0,  700.0,  700.0]
# ]

THRESHOLDS = [
    [38406.8, 49937.3, 75488.9],
    [20396.7, 27556.0, 44790.9],
    [22969.2, 30980.6, 47308.5],
    [28157.5, 32627.5, 42820.8],
    [13922.7, 17090.8, 26576.7]
]

HYSTERESIS = [
    [1878.9, 9429.3, 14966.5],
    [1488.4, 5537.5, 11153.6],
    [3367.6, 3961.3, 12128.7],
    [3310.0, 935.5, 9082.1],
    [927.4, 2185.0, 7134.9]
]

# --- DATA STRUCTURES ---
times = deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE)
data_r = [deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE) for _ in range(NUM_FLEX)]
data_s = [deque([0]*WINDOW_SIZE, maxlen=WINDOW_SIZE) for _ in range(NUM_FLEX)]

# Track dynamic min/max for auto-scaling Y-limits. 
# Initialized to threshold boundaries so the thresholds are always visible on screen.
# Updated to pull index 0 and 2 from the 2D HYSTERESIS array
y_min_track = [THRESHOLDS[i][0] - HYSTERESIS[i][0] - 2000 for i in range(NUM_FLEX)]
y_max_track = [THRESHOLDS[i][2] + HYSTERESIS[i][2] + 2000 for i in range(NUM_FLEX)]

is_running = True
start_time = time.time()

# --- BACKGROUND THREAD: SERIAL READER ---
def read_serial_data():
    global is_running
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        ser.flushInput()
        print(f"Connected to {SERIAL_PORT}. Receiving data...")
        
        while is_running:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: continue
            
            parts = line.split(',')
            if len(parts) == NUM_FLEX * 2:
                try:
                    vals = [float(p) for p in parts]
                    
                    if any(math.isinf(v) or math.isnan(v) for v in vals):
                        continue

                    current_time = time.time() - start_time
                    times.append(current_time)
                    
                    for i in range(NUM_FLEX):
                        data_r[i].append(vals[i*2])      # Resistance
                        data_s[i].append(vals[i*2 + 1])  # State
                        
                except ValueError:
                    pass 
    except Exception as e:
        print(f"Serial Error: {e}")
        is_running = False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()

# --- FOREGROUND THREAD: LIVE PLOTTING ---
fig, axs = plt.subplots(NUM_FLEX, 1, figsize=(12, 14), sharex=True)
fig.suptitle('Real-Time Resistance, Thresholds, & Quantized State', fontsize=14)

lines_r = []
lines_s = []
colors = ['blue', 'orange', 'green', 'red', 'purple']

for i in range(NUM_FLEX):
    # Left Axis (Resistance)
    line_r, = axs[i].plot([], [], lw=2, color=colors[i], label=f'R (Ohms)')
    axs[i].set_ylabel('Resistance (Ω)', color=colors[i])
    axs[i].tick_params(axis='y', labelcolor=colors[i])
    axs[i].grid(True, alpha=0.3)
    
    # Plot Thresholds and Hysteresis Deadbands
    for t_idx, threshold in enumerate(THRESHOLDS[i]):
        label = 'Threshold/Hyst' if t_idx == 0 else ""
        h_val = HYSTERESIS[i][t_idx] # Pull specific 2D hysteresis value
        
        # Draw threshold dashed line
        axs[i].axhline(y=threshold, color='black', linestyle='--', alpha=0.6, linewidth=1, label=label)
        # Shade hysteresis margin
        axs[i].axhspan(threshold - h_val, threshold + h_val, color='gray', alpha=0.2)
    
    # Right Axis (State)
    ax_right = axs[i].twinx()
    line_s, = ax_right.plot([], [], lw=2, color='black', linestyle='-', alpha=0.6, label='State')
    ax_right.set_ylabel('State (0-3)', color='black')
    ax_right.set_ylim(-0.5, 3.5)
    ax_right.set_yticks([0, 1, 2, 3]) 
    
    # Legends
    lines_combined = [line_r, line_s]
    labels_combined = [l.get_label() for l in lines_combined]
    axs[i].legend(lines_combined, labels_combined, loc='upper left')

    lines_r.append(line_r)
    lines_s.append(line_s)

axs[-1].set_xlabel('Time (seconds)')

def update_plot(frame):
    if not is_running:
        return lines_r + lines_s
        
    for i in range(NUM_FLEX):
        lines_r[i].set_data(times, data_r[i])
        lines_s[i].set_data(times, data_s[i])
        
        # 1. Update X-Axis (Time scroll)
        if len(times) > 1 and times[-1] > times[0]:
            axs[i].set_xlim(times[0], times[-1])
            
        # 2. Dynamic Auto-Scaling for Y-Axis
        if len(data_r[i]) > 0:
            current_max = max(data_r[i])
            current_min = min(data_r[i])
            
            # Update the tracked bounds if new records are hit
            if current_max > y_max_track[i]:
                y_max_track[i] = current_max
            if current_min < y_min_track[i]:
                y_min_track[i] = current_min
                
            # Add a 10% padding margin to the top and bottom
            margin = (y_max_track[i] - y_min_track[i]) * 0.1
            if margin < 1000: margin = 1000 # Minimum padding
            
            axs[i].set_ylim(y_min_track[i] - margin, y_max_track[i] + margin)
            
    return lines_r + lines_s

ani = animation.FuncAnimation(fig, update_plot, interval=20, blit=False, cache_frame_data=False)

plt.tight_layout()
plt.show()

print("Closing application...")
is_running = False
thread.join()