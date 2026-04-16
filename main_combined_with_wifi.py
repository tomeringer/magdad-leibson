#!/usr/bin/env python3
import os, time, math, sys, tty, termios, pigpio
import serial
import socket
import select
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

import chassis
import gripper
import arm
import vision
import piano_player 

os.environ['PIGPIO_ADDR'] = 'localhost'

# ========================================================
# CONSTANTS & CONFIGURATION
# ========================================================
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200

UDP_IP = "0.0.0.0"
UDP_PORT = 4210

ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06

factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

_arm_dir = 0
_prev_f = [0, 0, 0, 0, 0]
_binary_f = [0, 0, 0, 0, 0]  # Tracks the latched binary states of the fingers
_drive_hist = ["STOP", "STOP"]
_ignore_ultra = [False, None]
last_rx = 0.0

# ========================================================
# GRIPPER MODE
# ========================================================
def get_average_distance():
    dist_l = abs(chassis.enc_left.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    dist_r = abs(chassis.enc_right.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    return (dist_l + dist_r) / 2.0

def drive_arc(target_x, target_z):
    R = (target_x ** 2 + target_z ** 2) / (2 * abs(target_x))
    r_in, r_out = R - (chassis.TRACK_WIDTH_CM / 2), R + (chassis.TRACK_WIDTH_CM / 2)
    s_in = ARC_MAX_OUTER_SPEED * (r_in / r_out)
    v_l, v_r = (ARC_MAX_OUTER_SPEED, s_in) if target_x > 0 else (s_in, ARC_MAX_OUTER_SPEED)
    total_arc = R * math.atan2(target_z, R - abs(target_x)) - ARC_STOP_OFFSET_CM

    chassis.RIGHT_RPWM.value, chassis.LEFT_RPWM.value = v_r * ARC_SPEED_DIFF_FACTOR, v_l
    chassis.RIGHT_LPWM.value = chassis.LEFT_LPWM.value = 0.0
    chassis.enc_left.zero()
    chassis.enc_right.zero()
    while get_average_distance() < total_arc: time.sleep(0.01)
    chassis.stop_drive()

def bring_bottle_xz():
    gripper.move_step(0)
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 5.0:
        res = vision.detect_bottle_once()
        if res["found"]:
            drive_arc(res['X'], res['Z'])
            time.sleep(0.3)
            gripper.move_step(1)
            return
        time.sleep(0.1)

def handle_payload(merged_byte, flex_low, b3=0, b4=0):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _prev_f, _binary_f

    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03
    modeBit = (merged_byte >> 6) & 0x01

    # Extract raw 0-3 states
    f_2b = [(flex_low >> (i * 2)) & 0x03 for i in range(4)]
    f_2b.append((merged_byte >> 4) & 0x03)
    
    # FIXED: Software Hysteresis - Latch ON at 3, Latch OFF at 1 (or 0)
    for i in range(5):
        if _binary_f[i] == 0 and f_2b[i] == 3:
            _binary_f[i] = 1
        elif _binary_f[i] == 1 and f_2b[i] <= 1:
            _binary_f[i] = 0

    f = list(_binary_f) # Use the locked binary states for logic

    if all(f) and not all(_prev_f):
        chassis.stop_drive()
        arm.stop()
        bring_bottle_xz()
    else:
        if f[4] and not _prev_f[4]: gripper.move_step(1)
        if f[0] and not _prev_f[0]: gripper.move_step(0)
        _arm_dir = -1 if (f[2] and not f[1]) else 1 if (f[1] and not f[2]) else 0
        if _arm_dir != 0:
            gripper.move_step(_arm_dir > 1)

        # Drive Logic: Pitch/Roll
    req = "STOP"
    if pitchBits == 0b10:     req = "FWD"
    elif pitchBits == 0b01:   req = "REV"
    elif rollBits == 0b10:    req = "RIGHT"
    elif rollBits == 0b01:    req = "LEFT"

    

    too_close = chassis.obstacle_too_close()

    if _ignore_ultra[0] and (req in ("STOP", "REV") or req != _ignore_ultra[1]):
        _ignore_ultra[0] = False

    if (not _ignore_ultra[0]) and req in {"FWD", "LEFT", "RIGHT"}:
        if _drive_hist[1] == "STOP" and _drive_hist[0] == req:
            _ignore_ultra[:] = [True, req]

    ign = (_ignore_ultra[0] and req == _ignore_ultra[1])

    if req == "REV": chassis.drive_reverse()
    elif req == "FWD": chassis.drive_forward() if (not too_close or ign) else chassis.stop_drive()
    elif req == "LEFT": chassis.turn_left() if (not too_close or ign) else chassis.stop_drive()
    elif req == "RIGHT": chassis.turn_right() if (not too_close or ign) else chassis.stop_drive()
    else: chassis.stop_drive()

    if req != _drive_hist[1]: _drive_hist = [_drive_hist[1], req]

    last_rx = time.time()
    _prev_f = f

    print(f"[RX] R: {rollBits:02b} | P: {pitchBits:02b} | F(Bin): {f} | Cmd: {req} | Close: {too_close} | Ign: {ign}")

def run_ssh_control():
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try: tty.setraw(fd); return sys.stdin.read(1)
        finally: termios.tcsetattr(fd, termios.TCSADRAIN, old)

    print("WASD=Drive, R/F=Arm, O/C=Claw, Z=Auto, Q=Quit")
    while True:
        chassis.ultrasonic_tick()
        c = getch().lower()
        if c == 'w': chassis.drive_forward()
        elif c == 's': chassis.drive_reverse()
        elif c == 'a': chassis.turn_left()
        elif c == 'd': chassis.turn_right()
        elif c == 'r': arm.run(True)
        elif c == 'f': arm.run(False)
        elif c == 'o': gripper.move_step(0)
        elif c == 'c': gripper.move_step(1)
        elif c == 'z': bring_bottle_xz()
        elif c == 'p': print(vision.detect_bottle_once())
        elif c == ' ' or c == 'k': chassis.stop_drive(); arm.stop()
        elif c == 'q': break

# ========================================================
# HAND MODE
# ========================================================
def handle_hand_payload(merged_byte, flex_low, *args):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _binary_f
    
    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03
    modeBit = (merged_byte >> 6) & 0x01
    
    # Extract raw 0-3 states
    f_2b = [(flex_low >> (i * 2)) & 0x03 for i in range(4)]
    f_2b.append((merged_byte >> 4) & 0x03)

    # FIXED: Software Hysteresis - Latch ON at 3, Latch OFF at 1 (or 0)
    for i in range(5):
        if _binary_f[i] == 0 and f_2b[i] == 3:
            _binary_f[i] = 1
        elif _binary_f[i] == 1 and f_2b[i] <= 1:
            _binary_f[i] = 0

    if modeBit == 1:
        # --- MODE 1: HAND/SERVO CONTROL ---
        # Update fingers using RAW 0-3 states for smooth servo sweeps
        piano_player.set_states(f_2b)
        
        chassis.stop_drive()
        _arm_dir = 0
        req = "STOP"
        too_close = False
        ign = False
        
    else:
        # --- MODE 0: DRIVE & ARM CONTROL ---
        
        # Arm Logic: F1 up, F2 down, using the solid BINARY states
        if _binary_f[1] == 1:
            _arm_dir = 1
        elif _binary_f[2] == 1:
            _arm_dir = -1
        else:
            _arm_dir = 0
            
        # Drive Logic: Pitch/Roll
        req = "STOP"
        if pitchBits == 0b10:     req = "FWD"
        elif pitchBits == 0b01:   req = "REV"
        elif rollBits == 0b10:    req = "RIGHT"
        elif rollBits == 0b01:    req = "LEFT"
            
        too_close = chassis.obstacle_too_close()
        
        if _ignore_ultra[0] and (req in ("STOP", "REV") or req != _ignore_ultra[1]): 
            _ignore_ultra[0] = False
            
        if (not _ignore_ultra[0]) and req in {"FWD", "LEFT", "RIGHT"}:
            if _drive_hist[1] == "STOP" and _drive_hist[0] == req: 
                _ignore_ultra[:] = [True, req]

        ign = (_ignore_ultra[0] and req == _ignore_ultra[1])
        
        if req == "REV": chassis.drive_reverse()
        elif req == "FWD": chassis.drive_forward() if (not too_close or ign) else chassis.stop_drive()
        elif req == "LEFT": chassis.turn_left() if (not too_close or ign) else chassis.stop_drive()
        elif req == "RIGHT": chassis.turn_right() if (not too_close or ign) else chassis.stop_drive()
        else: chassis.stop_drive()
            
        if req != _drive_hist[1]: _drive_hist = [_drive_hist[1], req]
            
    last_rx = time.time()
    
    print(f"[RX] MODE: {modeBit} | R: {rollBits:02b} P: {pitchBits:02b} | F(Raw): {f_2b} | Cmd: {req} | Arm: {_arm_dir}")

def run_hand_ssh_control():
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try: tty.setraw(fd); return sys.stdin.read(1)
        finally: termios.tcsetattr(fd, termios.TCSADRAIN, old)

    print("WASD=Drive, 1-5=Cycle Fingers, Space/K=Stop, Q=Quit")
    fingers = [0, 0, 0, 0, 0]
    
    while True:
        chassis.ultrasonic_tick()
        c = getch().lower()
        if c == 'w': chassis.drive_forward()
        elif c == 's': chassis.drive_reverse()
        elif c == 'a': chassis.turn_left()
        elif c == 'd': chassis.turn_right()
        elif c in ['1', '2', '3', '4', '5']:
            idx = int(c) - 1
            fingers[idx] = (fingers[idx] + 1) % 4
            piano_player.set_states(fingers)
            print(f"Fingers state updated: {fingers}") 
        elif c == ' ' or c == 'k': chassis.stop_drive()
        elif c == 'q': break

# ========================================================
# MAIN
# ========================================================
if __name__ == "__main__":
    try:
        while True:
            mode = input("\nSelect mode: GRIPPER or HAND? (g/h/q to quit)\n").strip().lower()
            if mode == 'q': break

            elif mode == "g":
                try:
                    chassis.init(factory, pi_enc)
                    gripper.init(factory)
                    arm.init(factory)
                    vision.init()
                    
                    ctrl_mode = input("Select control: Keyboard (k), Arduino (a), or WiFi (w)?\n").strip().lower()
                    
                    if ctrl_mode == "k":
                        run_ssh_control()
                        
                    elif ctrl_mode == "w":
                        print(f"[WIFI] Starting UDP Receiver on port {UDP_PORT}...")
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        sock.bind((UDP_IP, UDP_PORT))
                        sock.settimeout(0.01)
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            if _arm_dir != 0: arm.run(_arm_dir == 1)
                            else: arm.stop()
                                
                            packet_received = False
                            latest_data = None
                            
                            while True:
                                try:
                                    data, addr = sock.recvfrom(1024)
                                    latest_data = data
                                    packet_received = True
                                except socket.timeout:
                                    break
                                except BlockingIOError:
                                    break
                            
                            if packet_received and latest_data:
                                if len(latest_data) >= 4 and latest_data[0] == 0xAA and latest_data[3] == 0x55:
                                    handle_payload(latest_data[1], latest_data[2])
                                elif len(latest_data) >= 2:
                                    handle_payload(latest_data[0], latest_data[1])
                                    
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                print("[WARNING] Connection Lost. Forcing ZERO payload.")
                                handle_payload(0, 0)
                                time.sleep(0.5)
                                
                    else:
                        print(f"[SERIAL] Starting Serial Receiver on {SERIAL_PORT}...")
                        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                        time.sleep(1)
                        ser.reset_input_buffer()
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            if _arm_dir != 0: arm.run(_arm_dir == 1)
                            else: arm.stop()
                                
                            while ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line:
                                    if line.startswith("DATA:"):
                                        try:
                                            parts = line.split(":")[1].split(",")
                                            if len(parts) >= 2:
                                                handle_payload(int(parts[0]), int(parts[1]))
                                        except Exception: pass
                                    else:
                                        print(f"[ARDUINO MSG] {line}")
                                        
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                print("[WARNING] Connection Lost. Forcing ZERO payload.")
                                handle_payload(0, 0) 
                                time.sleep(0.5) 
                            time.sleep(0.01)
                            
                except KeyboardInterrupt: print("\n[INFO] Returning to main menu...")
                except Exception as e: print(f"[ERROR] GRIPPER mode: {e}")
                finally:
                    chassis.stop_drive(); chassis.close_pins(); gripper.close_pins()
                    arm.close_pins(); vision.shutdown()
                    if 'ser' in locals() and hasattr(ser, 'is_open') and ser.is_open: ser.close()
                    if 'sock' in locals(): sock.close()
                        
            elif mode == "h":
                try:
                    chassis.init(factory, pi_enc)
                    piano_player.init(factory)
                    arm.init(factory)

                    ctrl_mode = input("Select control: Keyboard (k), Arduino (a), or WiFi (w)?\n").strip().lower()

                    if ctrl_mode == "k":
                        run_hand_ssh_control()
                        
                    elif ctrl_mode == "w":
                        print(f"[WIFI] Starting UDP Receiver on port {UDP_PORT}...")
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        sock.bind((UDP_IP, UDP_PORT))
                        sock.settimeout(0.01)
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            
                            if _arm_dir != 0: arm.run(_arm_dir == 1)
                            else: arm.stop()
                            
                            packet_received = False
                            latest_data = None
                            
                            while True:
                                try:
                                    data, addr = sock.recvfrom(1024)
                                    latest_data = data
                                    packet_received = True
                                except socket.timeout:
                                    break
                                except BlockingIOError:
                                    break
                                    
                            if packet_received and latest_data:
                                if len(latest_data) >= 4 and latest_data[0] == 0xAA and latest_data[3] == 0x55:
                                    handle_hand_payload(latest_data[1], latest_data[2])
                                elif len(latest_data) >= 2:
                                    handle_hand_payload(latest_data[0], latest_data[1])
                                    
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                _arm_dir = 0
                                piano_player.set_states([0,0,0,0,0])
                                print("[WARNING] Connection Lost. Forcing ZERO payload.")
                                handle_hand_payload(0, 0) 
                                time.sleep(0.5) 
                                
                    else: 
                        print(f"[SERIAL] Starting Serial Receiver on {SERIAL_PORT}...")
                        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                        time.sleep(1)
                        ser.reset_input_buffer()
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            
                            if _arm_dir != 0: arm.run(_arm_dir == 1)
                            else: arm.stop()
                            
                            while ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line:
                                    if line.startswith("DATA:"):
                                        try:
                                            parts = line.split(":")[1].split(",")
                                            if len(parts) >= 2:
                                                handle_hand_payload(int(parts[0]), int(parts[1]))
                                        except Exception: pass
                                    else:
                                        print(f"[ARDUINO MSG] {line}")
                                        
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                _arm_dir = 0
                                piano_player.set_states([0,0,0,0,0]) 
                                print("[WARNING] Connection Lost. Forcing ZERO payload.")
                                handle_hand_payload(0, 0) 
                                time.sleep(0.5) 
                            time.sleep(0.01)

                except KeyboardInterrupt: print("\n[INFO] Returning to main menu...")
                except Exception as e: print(f"[ERROR] HAND mode: {e}")
                finally:
                    chassis.stop_drive(); chassis.close_pins(); piano_player.close_pins()
                    arm.close_pins() 
                    if 'ser' in locals() and hasattr(ser, 'is_open') and ser.is_open: ser.close()
                    if 'sock' in locals(): sock.close()
                    
            else: print("Invalid input.")

    except KeyboardInterrupt: print("\n[INFO] Interrupted by user.")
    finally:
        chassis.stop_drive()
        try: vision.shutdown()
        except Exception: pass 
        GPIO.cleanup()
        print("[SYSTEM] Safely powered down.")