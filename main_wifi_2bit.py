import os
import time
import socket
import pigpio
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

# Import your robot modules
import chassis
import piano_player

os.environ['PIGPIO_ADDR'] = 'localhost'

# ========================================================
# CONFIGURATION
# ========================================================
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

# State tracking for chassis
_drive_hist = ["STOP", "STOP"]
_ignore_ultra = [False, None]
last_rx = 0.0

def setup_hardware():
    factory = PiGPIOFactory()
    pi_enc = pigpio.pi()
    GPIO.setmode(GPIO.BCM)
    
    print("[SYSTEM] Initializing Chassis...")
    chassis.init(factory, pi_enc)
    
    print("[SYSTEM] Initializing Piano Player...")
    piano_player.init(factory)
    
    return factory, pi_enc

def handle_payload(merged_byte, flex_low):
    global _drive_hist, _ignore_ultra, last_rx
    
    # --- 1. DECODE IMU DATA (From merged_byte) ---
    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03
    
    # --- 2. DECODE FLEX DATA (From both bytes) ---
    f4 = (merged_byte >> 4) & 0x03
    
    f0 = flex_low & 0x03
    f1 = (flex_low >> 2) & 0x03
    f2 = (flex_low >> 4) & 0x03
    f3 = (flex_low >> 6) & 0x03

    # Pass all 5 fingers to the piano player
    piano_player.set_states([f0, f1, f2, f3, f4])

    # --- 3. DRIVE CHASSIS WITH ULTRASONIC SAFETY ---
    req = "STOP"
    if pitchBits == 0b01:     # + Pitch
        req = "FWD"
    elif pitchBits == 0b10:   # - Pitch
        req = "REV"
    elif rollBits == 0b01:    # + Roll
        req = "RIGHT"
    elif rollBits == 0b10:    # - Roll
        req = "LEFT"
        
    too_close = chassis.obstacle_too_close()
    
    if _ignore_ultra[0] and (req in ("STOP", "REV") or req != _ignore_ultra[1]): 
        _ignore_ultra[0] = False
        
    if (not _ignore_ultra[0]) and req in {"FWD", "LEFT", "RIGHT"}:
        if _drive_hist[1] == "STOP" and _drive_hist[0] == req: 
            _ignore_ultra[:] = [True, req]

    ign = (_ignore_ultra[0] and req == _ignore_ultra[1])
    
    if req == "REV":
        chassis.drive_reverse()
    elif req == "FWD":
        chassis.drive_forward() if (not too_close or ign) else chassis.stop_drive()
    elif req == "LEFT":
        chassis.turn_left() if (not too_close or ign) else chassis.stop_drive()
    elif req == "RIGHT":
        chassis.turn_right() if (not too_close or ign) else chassis.stop_drive()
    else:
        chassis.stop_drive()
        
    # FIX: Only update the history buffer if the command actually changed.
    # This prevents the 10Hz UDP stream from filling the buffer with ["STOP", "STOP"] 
    # while your hand is resting in neutral.
    if req != _drive_hist[1]:
        _drive_hist = [_drive_hist[1], req]
        
    last_rx = time.time()

    # Debug print (can be commented out in production)
    print(f"\r[RX] RollBits: {rollBits:02b} | PitchBits: {pitchBits:02b} | Flex: {[f0, f1, f2, f3, f4]} | DriveCmd: {req} | TooClose: {too_close} | IgnoringUltra: {ign}      ", end="")

def main():
    setup_hardware()
    print(f"[NETWORK] Starting UDP Server on {UDP_IP}:{UDP_PORT}...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(0.02)
    if sock is None:
        print("[ERROR] Failed to create UDP socket. Exiting.")
        return
    print("[NETWORK] UDP Server initialized successfully.")
    
    global last_rx
    last_rx = time.time()

    print("[SYSTEM] Robot Ready. Waiting for raw 2-byte glove commands...")

    try:
        while True:
            # Continuously check sensors
            chassis.ultrasonic_tick()
            
            try:
                # Attempt to read UDP packet
                data, addr = sock.recvfrom(1024)
                
                # Validate that we received at least the 2 bytes we expect
                if len(data) >= 2:
                    handle_payload(data[0], data[1])
                    
            except socket.timeout:
                # If no packet arrives for 0.7 seconds, halt the robot
                if time.time() - last_rx > 0.7: 
                    chassis.stop_drive()

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down...")
    finally:
        chassis.stop_drive()
        sock.close()
        GPIO.cleanup()
        print("[SYSTEM] Safely powered down.")

if __name__ == "__main__":
    main()