import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import math

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM6'  # <-- Update to your ESP32 port
BAUD_RATE = 115200
NUM_FINGERS = 5
STATES = [0, 1, 2, 3]
STATE_NAMES = ["0 (Flat/0°)", "1 (20°)", "2 (40°)", "3 (Bent/60°)"]

# Added Pin Names so the generated arrays are commented perfectly
PIN_NAMES = ["A3", "A0", "A2", "A7", "A1"]

data = {f: {s: [] for s in STATES} for f in range(NUM_FINGERS)}

def collect_data():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
        time.sleep(2)
        print("Connected. Ready for Quartile Calibration.\n")

        for state in STATES:
            input(f"ACTION: Hold all fingers at {STATE_NAMES[state]}.\nPress ENTER to capture 1000 samples...")
            ser.write(str(state).encode('utf-8'))
            
            count = 0
            while True:
                line = ser.readline().decode('utf-8').strip()
                if line == "DONE": break
                
                if line:
                    parts = line.split(',')
                    if len(parts) == NUM_FINGERS:
                        valid_line = True
                        temp_vals = []
                        
                        # Validate the entire row before appending
                        for i in range(NUM_FINGERS):
                            try:
                                val = float(parts[i])
                                if math.isinf(val) or math.isnan(val):
                                    valid_line = False
                                    break
                                temp_vals.append(val)
                            except ValueError:
                                valid_line = False
                                break
                                
                        # Only save the data if all 5 sensors reported clean numbers
                        if valid_line:
                            for i in range(NUM_FINGERS):
                                data[i][state].append(temp_vals[i])
                            count += 1
                            if count % 200 == 0: 
                                print(f"Captured {count}/1000...")
            
            print(f"--- {STATE_NAMES[state]} Complete ---\n")

        ser.close()
        analyze_and_generate_code()

    except Exception as e:
        print(f"ERROR: {e}")

def analyze_and_generate_code():
    print("\n========================================================")
    print("                QUARTILE ANALYSIS RESULTS             ")
    print("========================================================")
    
    thresholds_cpp = "float THRESHOLDS[NUM_FLEX][3] = {\n"
    hysteresis_cpp = "float HYSTERESIS[NUM_FLEX][3] = {\n"
    
    fig, axes = plt.subplots(NUM_FINGERS, 1, figsize=(10, 15), sharex=False)
    colors = ['#1f77b4', '#2ca02c', '#ff7f0e', '#d62728']
    
    for i in range(NUM_FINGERS):
        # Arrays to hold the data for easy percentiling
        arr_0 = np.array(data[i][0])
        arr_1 = np.array(data[i][1])
        arr_2 = np.array(data[i][2])
        arr_3 = np.array(data[i][3])

        # 1. Plot histograms with medians
        for state, arr in enumerate([arr_0, arr_1, arr_2, arr_3]):
            if len(arr) > 0:
                med = np.median(arr)
                axes[i].hist(arr, bins=30, alpha=0.5, color=colors[state], 
                             label=f"{STATE_NAMES[state]} (Median={med:.0f})")

        # 2. Calculate the Percentiles (Upper 4th / Lower 4th)
        # state 0: upper 4th
        q3_0 = np.percentile(arr_0, 75) if len(arr_0) > 0 else 0
        
        # state 1: lower 4th and upper 4th
        q1_1 = np.percentile(arr_1, 25) if len(arr_1) > 0 else 0
        q3_1 = np.percentile(arr_1, 75) if len(arr_1) > 0 else 0
        
        # state 2: lower 4th and upper 4th
        q1_2 = np.percentile(arr_2, 25) if len(arr_2) > 0 else 0
        q3_2 = np.percentile(arr_2, 75) if len(arr_2) > 0 else 0
        
        # state 3: lower 4th
        q1_3 = np.percentile(arr_3, 25) if len(arr_3) > 0 else 0

        # 3. Calculate the Optimal Intersection Thresholds (Middle between the quarters)
        T1 = (q3_0 + q1_1) / 2.0
        T2 = (q3_1 + q1_2) / 2.0
        T3 = (q3_2 + q1_3) / 2.0
        
        # 4. Calculate Individual Hysteresis (Half the length between the quarters)
        h1 = (q1_1 - q3_0) / 2.0
        h2 = (q1_2 - q3_1) / 2.0
        h3 = (q1_3 - q3_2) / 2.0

        # Safety check: Prevent negative or extremely tiny hysteresis
        h1 = max(h1, 100.0)
        h2 = max(h2, 100.0)
        h3 = max(h3, 100.0)

        # Plot vertical lines for thresholds and shades for exact hysteresis gaps
        for T, H in zip([T1, T2, T3], [h1, h2, h3]):
            axes[i].axvline(x=T, color='black', linestyle='--', linewidth=1.5)
            axes[i].axvspan(T - H, T + H, color='gray', alpha=0.2)
            
        # Updated Title to include Pin name
        axes[i].set_title(f"Finger {i} ({PIN_NAMES[i]}) Quartile Distributions")
        axes[i].legend(loc="upper left", fontsize='small')
        
        # Format C++ Output with commented Pin labels
        thresholds_cpp += f"  {{{T1:.1f}f, {T2:.1f}f, {T3:.1f}f}}"
        thresholds_cpp += f", // f{i} ({PIN_NAMES[i]})\n" if i < NUM_FINGERS - 1 else f"  // f{i} ({PIN_NAMES[i]})\n"
        
        hysteresis_cpp += f"  {{{h1:.1f}f, {h2:.1f}f, {h3:.1f}f}}"
        hysteresis_cpp += f", // f{i} ({PIN_NAMES[i]})\n" if i < NUM_FINGERS - 1 else f"  // f{i} ({PIN_NAMES[i]})\n"

    thresholds_cpp += "};\n"
    hysteresis_cpp += "};\n"
    
    print("\n>>> COPY AND PASTE THIS INTO YOUR ESP32 CODE <<<\n")
    print(thresholds_cpp)
    print(hysteresis_cpp)
    print("\n========================================================")
    
    plt.tight_layout(pad=3.0)
    plt.show()

if __name__ == '__main__':
    collect_data()