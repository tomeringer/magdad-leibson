import socket
import time
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import DistanceSensor

# Import all robot modules
import piano_player as piano
import chassis
import arm
import gripper

# ==========================================
#              NETWORK SETUP
# ==========================================
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening for Dual-Mode Glove Data on UDP port {UDP_PORT}...")

# ==========================================
#            HARDWARE SETUP
# ==========================================
factory = PiGPIOFactory()

# Initialize all components
piano.init(factory)
chassis.init(factory)
arm.init(factory)
gripper.init(factory)

# Ultrasonic setup (Ensure these pins match your physical wiring)
ECHO_PIN = 24
TRIG_PIN = 23
ultrasonic = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN, pin_factory=factory)
MIN_SAFE_DISTANCE = 0.20 # 20 cm stop distance

# Keep track of the current mode to avoid spamming the console
current_mode = -1 

# ==========================================
#               MAIN LOOP
# ==========================================
try:
    while True:
        data, addr = sock.recvfrom(1024)
        
        if len(data) == 2:
            merged_byte = data[0]
            flex_low = data[1]
            
            # --- 1. PARSE THE BITS ---
            # Extract Mode Bit (Bit 6)
            mode_bit = (merged_byte >> 6) & 0x01
            
            # Extract IMU Bits (Roll: Bits 2-3 | Pitch: Bits 0-1)
            roll_bits = (merged_byte >> 2) & 0x03
            pitch_bits = merged_byte & 0x03
            
            # Extract Finger States (0 to 3)
            f4_state = (merged_byte >> 4) & 0x03
            f3_state = (flex_low >> 6) & 0x03
            f2_state = (flex_low >> 4) & 0x03
            f1_state = (flex_low >> 2) & 0x03
            f0_state = flex_low & 0x03

            # Print mode switch for debugging
            if mode_bit != current_mode:
                print(f"\n>> SWITCHED TO MODE: {'PIANO (1)' if mode_bit == 1 else 'DRIVE/ARM (0)'}")
                current_mode = mode_bit

            # --- 2. EXECUTE BEHAVIOR BASED ON MODE ---
            if mode_bit == 1:
                # ==========================================
                #          MODE 1: PIANO MODE
                # ==========================================
                # Safety first: Stop the chassis if we just switched to piano mode
                chassis.stop()
                
                # Pass all 5 finger states to the piano player
                piano.set_states([f0_state, f1_state, f2_state, f3_state, f4_state])

            elif mode_bit == 0:
                # ==========================================
                #       MODE 0: DRIVE & ARM MODE
                # ==========================================
                
                # --- A. Arm & Gripper Control (Fingers 1 & 2) ---
                # Exact mapping from main.py. Adjust state thresholds if needed!
                if f1_state >= 2: # If Finger 1 is heavily bent
                    arm.down()    # (Or arm.move_down() based on your module's naming)
                else:
                    arm.up()      # (Or arm.move_up())

                if f2_state >= 2: # If Finger 2 is heavily bent
                    gripper.close()
                else:
                    gripper.open()

                # --- B. IMU Chassis Control & Ultrasonic Logic ---
                # Check distance for forward obstacle avoidance
                distance = ultrasonic.distance
                obstacle_ahead = distance < MIN_SAFE_DISTANCE

                # Pitch drives Forward/Backward, Roll drives Left/Right
                # 0b01 (1) = Forward/Right | 0b10 (2) = Backward/Left | 0b00 (0) = Neutral
                
                if pitch_bits == 1:       # Forward
                    if not obstacle_ahead:
                        chassis.forward()
                    else:
                        chassis.stop()    # Ultrasonic override
                        
                elif pitch_bits == 2:     # Backward
                    chassis.backward()
                    
                elif roll_bits == 1:      # Right
                    chassis.right()
                    
                elif roll_bits == 2:      # Left
                    chassis.left()
                    
                else:                     # Neutral flat hand
                    chassis.stop()

except KeyboardInterrupt:
    print("\nShutting down gracefully...")
    chassis.stop()
    sock.close()