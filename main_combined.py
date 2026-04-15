#!/usr/bin/env python3
import os, time, socket, math, sys, tty, termios, pigpio
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

# Import as namespaces to avoid NoneType copy errors
import chassis
import gripper
import arm
import vision
import piano_player # Added piano_player for Hand mode

os.environ['PIGPIO_ADDR'] = 'localhost'

# Constants for Gripper mode
ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
CALIBRATION_FILE_PATH = "Autonomy/stereo_calibration.pkl"

# Constants for Hand mode network
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

# Initial Setup (Shared)
factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

# Shared Global Variables
_arm_dir = 0
_prev_f = [0, 0, 0, 0]
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

    # Call through chassis namespace
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

def handle_payload(merged_byte, flex_low):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _prev_f

    # --- 1. DECODE IMU DATA (From merged_byte) ---
    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03

    # --- 2. DECODE FLEX DATA (From both bytes) ---
    f4 = (merged_byte >> 4) & 0x03

    f0 = flex_low & 0x03
    f1 = (flex_low >> 2) & 0x03
    f2 = (flex_low >> 4) & 0x03
    f3 = (flex_low >> 6) & 0x03

    f_2b = [f0, f1, f2, f3, f4]
    f = [1 if f == 3 else 0 for f in f_2b]

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
    if pitchBits == 0b01:  # + Pitch
        req = "FWD"
    elif pitchBits == 0b10:  # - Pitch
        req = "REV"
    elif rollBits == 0b01:  # + Roll
        req = "RIGHT"
    elif rollBits == 0b10:  # - Roll
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

    # Debug print (can be commented out in production)
    print(
        f"\r[RX] RollBits: {rollBits:02b} | PitchBits: {pitchBits:02b} | Flex: {[f0, f1, f2, f3, f4]} | DriveCmd: {req} | TooClose: {too_close} | IgnoringUltra: {ign}      ",
        end="")


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
        if c == 'w':
            chassis.drive_forward()
        elif c == 's':
            chassis.drive_reverse()
        elif c == 'a':
            chassis.turn_left()
        elif c == 'd':
            chassis.turn_right()
        elif c == 'r':
            arm.run(True)
        elif c == 'f':
            arm.run(False)
        elif c == 'o':
            gripper.move_step(0)
        elif c == 'c':
            gripper.move_step(1)
        elif c == 'z':
            bring_bottle_xz()
        elif c == 'p':
            print(vision.detect_bottle_once())
        elif c == ' ' or c == 'k':
            chassis.stop_drive(); arm.stop()
        elif c == 'q':
            break

# ========================================================
# HAND MODE FUNCTIONS
# ========================================================

def handle_hand_payload(merged_byte, flex_low):
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

    mode_bit = (merged_byte >> 6) & 0x01

    if mode_bit == 1:
        piano_player.set_states([f0, f1, f2, f3, f4])
        print(f"\r[RX] RollBits: {rollBits:02b} | PitchBits: {pitchBits:02b} | Flex: {[f0, f1, f2, f3, f4]}", end="")
    else:
        piano_player.set_states([0, 0, 0, 0, 0])
        if f1 > 2:
            arm.run(False)
        elif f2 > 2:
            arm.run(True)
        else:
            arm.stop()

        req = "STOP"
        if pitchBits == 0b10:     # + Pitch
            req = "FWD"
        elif pitchBits == 0b01:   # - Pitch
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
        print(f"\r[RX] RollBits: {rollBits:02b} | PitchBits: {pitchBits:02b} | Flex: {[f0, f1, f2, f3, f4]} | DriveCmd: {req} | TooClose: {too_close} | IgnoringUltra: {ign}      ", end="")


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
        
        if c == 'w':
            chassis.drive_forward()
        elif c == 's':
            chassis.drive_reverse()
        elif c == 'a':
            chassis.turn_left()
        elif c == 'd':
            chassis.turn_right()
        elif c in ['1', '2', '3', '4', '5']:
            idx = int(c) - 1
            # Cycle the specific finger state: 0 -> 1 -> 2 -> 3 -> 0
            fingers[idx] = (fingers[idx] + 1) % 4
            piano_player.set_states(fingers)
            print(f"\rFingers state updated: {fingers}        ", end="")
        elif c == ' ' or c == 'k':
            chassis.stop_drive()
        elif c == 'q':
            break

# ========================================================
# MAIN EXECUTION
# ========================================================
if __name__ == "__main__":
    try:
        # Main loop to keep the program running and ask for mode repeatedly
        while True:
            mode = input("\nSelect mode: GRIPPER or HAND? (g/h/q to quit)\n").strip().lower()

            if mode == 'q':
                break # Exit the main loop and proceed to final shutdown

            elif mode == "g":
                try:
                    # Initialize components for gripper mode
                    chassis.init(factory, pi_enc)
                    gripper.init(factory)
                    arm.init(factory)
                    vision.init()
                    
                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_ssh_control()
                    else:
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        sock.bind(("0.0.0.0", 4210))
                        sock.settimeout(0.02)
                        while True:
                            chassis.ultrasonic_tick()
                            if _arm_dir != 0:
                                arm.run(_arm_dir == 1)
                            else:
                                arm.stop()
                            try:
                                data, addr = sock.recvfrom(1024)
                                if len(data) >= 2:
                                    handle_payload(data[0], data[1])
                            except socket.timeout:
                                if time.time() - last_rx > 0.7: chassis.stop_drive()
                except KeyboardInterrupt:
                    print("\n[INFO] Returning to main menu...")
                except Exception as e:
                    print(f"[ERROR] An error occurred in GRIPPER mode: {e}")
                finally:
                    # Cleanup specific to gripper mode before looping back
                    chassis.stop_drive()
                    chassis.close_pins()
                    gripper.close_pins()
                    arm.close_pins()
                    vision.shutdown()
                    try: sock.close()
                    except Exception: pass
                        
            elif mode == "h":
                try:
                    # Initialize components for hand mode
                    chassis.init(factory, pi_enc)
                    arm.init(factory)
                    piano_player.init(factory)

                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_hand_ssh_control()
                    else: 
                        print(f"[NETWORK] Starting UDP Server on {UDP_IP}:{UDP_PORT}...")
                        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        sock.bind((UDP_IP, UDP_PORT))
                        sock.settimeout(0.02)
                        
                        print("[NETWORK] UDP Server initialized successfully.")
                        last_rx = time.time()
                        print("[SYSTEM] Robot Ready. Waiting for raw 2-byte glove commands...")
                        
                        while True:
                            chassis.ultrasonic_tick()
                            try:
                                data, addr = sock.recvfrom(1024)                    
                                if len(data) >= 2:
                                    handle_hand_payload(data[0], data[1])
                            except socket.timeout:
                                # If no packet arrives for 0.7 seconds, halt the robot
                                if time.time() - last_rx > 0.7: 
                                    chassis.stop_drive()                    
                except KeyboardInterrupt:
                    print("\n[INFO] Returning to main menu...")
                except Exception as e:
                    print(f"[ERROR] An error occurred in HAND mode: {e}")
                finally:
                    # Cleanup specific to hand mode before looping back
                    chassis.stop_drive()
                    chassis.close_pins()
                    piano_player.close_pins()
                    arm.close_pins()
                    try: sock.close()
                    except Exception: pass
                    
            else:
                print("Invalid input. Please select 'g', 'h', or 'q'.")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down completely...")
        
    finally:
        # Final safe shutdown for the entire system
        chassis.stop_drive()
        try:
            vision.shutdown()
        except Exception:
            pass # In case vision wasn't initialized 
            
        GPIO.cleanup()
        print("[SYSTEM] Safely powered down.")