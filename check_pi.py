#!/usr/bin/env python3
import os, time, math, sys, tty, termios, pigpio
import serial
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

# Import as namespaces to avoid NoneType copy errors
import chassis
import gripper
import arm
import vision
import piano_player # Added piano_player for Hand mode

os.environ['PIGPIO_ADDR'] = 'localhost'

# ========================================================
# CONSTANTS & CONFIGURATION
# ========================================================
# Serial (Arduino USB) Configuration
SERIAL_PORT = '/dev/ttyACM0'  # שנה ל-ttyUSB0 אם נדרש
BAUD_RATE = 115200

# Constants for Gripper mode
ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
CALIBRATION_FILE_PATH = "Autonomy/stereo_calibration.pkl"

# Initial Setup (Shared)
factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

# Shared Global Variables
_arm_dir = 0
_prev_f = [0, 0, 0, 0, 0]
_drive_hist = ["STOP", "STOP"]
_ignore_ultra = [False, None]
last_rx = 0.0

# ========================================================
# GRIPPER MODE FUNCTIONS
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

def handle_payload(b1, b2, b3, b4):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _prev_f

    # --- 1. DECODE IMU DATA ---
    rollBits = (b2 >> 2) & 0x03
    pitchBits = (b2 >> 4) & 0x03

    # --- 2. DECODE FLEX DATA ---
    flex_data = (b2 << 8) | b1
    f_2b = [(flex_data >> (i * 2)) & 0x03 for i in range(5)] # רשימה של 0-3
    f = [0 if (val <= 1) else 1 for val in f_2b] # 0/1 = ישר, 2/3 = מקופל

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

    if req != _drive_hist[1]:
        _drive_hist = [_drive_hist[1], req]

    last_rx = time.time()
    _prev_f = f

    print(f"\r[RX] Roll: {rollBits:02b} | Pitch: {pitchBits:02b} | Flex: {f_2b} | DriveCmd: {req} | TooClose: {too_close} | IgnUltra: {ign}      ", end="")

def run_ssh_control():
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd); return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

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
# HAND MODE FUNCTIONS
# ========================================================

def handle_hand_payload(b1, b2, b3, b4):
    global _drive_hist, _ignore_ultra, last_rx
    
    # --- 1. DECODE IMU DATA ---
    rollBits = (b2 >> 2) & 0x03
    pitchBits = (b2 >> 4) & 0x03
    
    # --- 2. DECODE FLEX DATA ---
    flex_data = (b2 << 8) | b1
    f_2b = [(flex_data >> (i * 2)) & 0x03 for i in range(5)]

    piano_player.set_states(f_2b)

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
        
    if req != _drive_hist[1]:
        _drive_hist = [_drive_hist[1], req]
        
    last_rx = time.time()

    print(f"\r[RX] Roll: {rollBits:02b} | Pitch: {pitchBits:02b} | Flex: {f_2b} | DriveCmd: {req} | TooClose: {too_close} | IgnUltra: {ign}      ", end="")

def run_hand_ssh_control():
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd); return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    print("WASD=Drive, 1-5=Cycle Fingers (0-3), Space/K=Stop, Q=Quit")
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
            print(f"\rFingers state updated: {fingers}        ", end="")
        elif c == ' ' or c == 'k': chassis.stop_drive()
        elif c == 'q': break

# ========================================================
# MAIN EXECUTION
# ========================================================
if __name__ == "__main__":
    try:
        while True:
            mode = input("\nSelect mode: GRIPPER or HAND? (g/h/q to quit)\n").strip().lower()

            if mode == 'q':
                break

            elif mode == "g":
                try:
                    chassis.init(factory, pi_enc)
                    gripper.init(factory)
                    arm.init(factory)
                    vision.init()
                    
                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_ssh_control()
                    else:
                        print(f"[SERIAL] Starting Serial Receiver on {SERIAL_PORT}...")
                        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
                        ser.dtr = False; time.sleep(1); ser.reset_input_buffer(); ser.dtr = True
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            
                            # הפעלת הזרוע ברקע
                            if _arm_dir != 0: arm.run(_arm_dir == 1)
                            else: arm.stop()
                                
                            # קריאה מצינור ה-USB (Arduino)
                            while ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line.startswith("DATA:"):
                                    try:
                                        parts = line.split(":")[1].split(",")
                                        if len(parts) == 4:
                                            b1, b2, b3, b4 = [int(x) for x in parts]
                                            handle_payload(b1, b2, b3, b4)
                                    except Exception:
                                        pass
                                        
                            # חיווי ניתוק ועצירת הרובוט
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                
                            time.sleep(0.01)
                            
                except KeyboardInterrupt:
                    print("\n[INFO] Returning to main menu...")
                except Exception as e:
                    print(f"[ERROR] An error occurred in GRIPPER mode: {e}")
                finally:
                    chassis.stop_drive()
                    chassis.close_pins()
                    gripper.close_pins()
                    arm.close_pins()
                    vision.shutdown()
                    if 'ser' in locals() and ser.is_open: ser.close()
                        
            elif mode == "h":
                try:
                    chassis.init(factory, pi_enc)
                    piano_player.init(factory)

                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_hand_ssh_control()
                    else: 
                        print(f"[SERIAL] Starting Serial Receiver on {SERIAL_PORT}...")
                        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
                        ser.dtr = False; time.sleep(1); ser.reset_input_buffer(); ser.dtr = True
                        last_rx = time.time()
                        
                        print("[SYSTEM] Robot Ready. Listening for 4-byte Glove Array...")
                        
                        while True:
                            chassis.ultrasonic_tick()
                            
                            while ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line.startswith("DATA:"):
                                    try:
                                        parts = line.split(":")[1].split(",")
                                        if len(parts) == 4:
                                            b1, b2, b3, b4 = [int(x) for x in parts]
                                            handle_hand_payload(b1, b2, b3, b4)
                                    except Exception:
                                        pass
                                        
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                
                            time.sleep(0.01)

                except KeyboardInterrupt:
                    print("\n[INFO] Returning to main menu...")
                except Exception as e:
                    print(f"[ERROR] An error occurred in HAND mode: {e}")
                finally:
                    chassis.stop_drive()
                    chassis.close_pins()
                    piano_player.close_pins()
                    arm.close_pins()
                    if 'ser' in locals() and ser.is_open: ser.close()
                    
            else:
                print("Invalid input. Please select 'g', 'h', or 'q'.")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down completely...")
        
    finally:
        chassis.stop_drive()
        try: vision.shutdown()
        except Exception: pass 
        GPIO.cleanup()
        print("[SYSTEM] Safely powered down.")
