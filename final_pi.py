#!/usr/bin/env python3
import os, time, math, sys, tty, termios, pigpio
import serial
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

ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06

factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

_arm_dir = 0
_prev_f = [0, 0, 0, 0, 0]
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

def handle_payload(merged_byte, flex_low):
    global _drive_hist, _ignore_ultra, last_rx, _arm_dir, _prev_f

    # Decode Compact Payload
    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03

    f4 = (merged_byte >> 4) & 0x03
    f0 = flex_low & 0x03
    f1 = (flex_low >> 2) & 0x03
    f2 = (flex_low >> 4) & 0x03
    f3 = (flex_low >> 6) & 0x03

    f_2b = [f0, f1, f2, f3, f4]
    f = [1 if val == 3 else 0 for val in f_2b]

    if all(f) and not all(_prev_f):
        chassis.stop_drive()
        arm.stop()
        bring_bottle_xz()
    else:
        if f[4] and not _prev_f[4]: gripper.move_step(1)
        if f[0] and not _prev_f[0]: gripper.move_step(0)
        _arm_dir = 1 if (f[2] and not f[1]) else -1 if (f[1] and not f[2]) else 0
        if _arm_dir != 0:
            gripper.move_step(_arm_dir > 1)

    req = "STOP"
    if pitchBits == 0b10:     req = "FWD"
    elif pitchBits == 0b01:   req = "REV"
    elif rollBits == 0b01:    req = "RIGHT"
    elif rollBits == 0b10:    req = "LEFT"

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

    print(f"[RX] R: {rollBits:02b} | P: {pitchBits:02b} | F: {f_2b} | Cmd: {req} | Close: {too_close} | Ign: {ign}")

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
def handle_hand_payload(merged_byte, flex_low):
    global _drive_hist, _ignore_ultra, last_rx
    
    rollBits = (merged_byte >> 2) & 0x03
    pitchBits = merged_byte & 0x03
    
    f4 = (merged_byte >> 4) & 0x03
    f0 = flex_low & 0x03
    f1 = (flex_low >> 2) & 0x03
    f2 = (flex_low >> 4) & 0x03
    f3 = (flex_low >> 6) & 0x03

    mode_bit = (merged_byte >> 6) & 0x01
    f_2b = [f0, f1, f2, f3, f4]

    if mode_bit == 1:
        piano_player.set_states(f_2b)
        print(f"[PIANO] R: {rollBits:02b} | P: {pitchBits:02b} | F: {f_2b}")
    else:
        piano_player.set_states([0, 0, 0, 0, 0])
        if f1 > 2: arm.run(False)
        elif f2 > 2: arm.run(True)
        else: arm.stop()

        req = "STOP"
        if pitchBits == 0b10:     req = "FWD"
        elif pitchBits == 0b01:   req = "REV"
        elif rollBits == 0b01:    req = "RIGHT"
        elif rollBits == 0b10:    req = "LEFT"
            
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
            
        print(f"[RX] R: {rollBits:02b} | P: {pitchBits:02b} | F: {f_2b} | Cmd: {req} | Close: {too_close} | Ign: {ign}")

    last_rx = time.time()

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
            fingers[idx] = 3 if fingers[idx] == 0 else 0
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
                    
                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_ssh_control()
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
                                
                            if ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line:
                                    if line.startswith("DATA:"):
                                        try:
                                            parts = line.split(":")[1].split(",")
                                            if len(parts) == 2:
                                                handle_payload(int(parts[0]), int(parts[1]))
                                        except Exception: pass
                                    else:
                                        print(f"[ARDUINO MSG] {line}")
                                        
                            # חסכון בסוללה ומניעת השתוללות במקרה ניתוק
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                print("[WARNING] Connection Lost. Halting...")
                                handle_payload(0, 0) 
                                time.sleep(0.5) 
                            time.sleep(0.01)
                            
                except KeyboardInterrupt: print("\n[INFO] Returning to main menu...")
                except Exception as e: print(f"[ERROR] GRIPPER mode: {e}")
                finally:
                    chassis.stop_drive(); chassis.close_pins(); gripper.close_pins()
                    arm.close_pins(); vision.shutdown()
                    if 'ser' in locals() and ser.is_open: ser.close()
                        
            elif mode == "h":
                try:
                    chassis.init(factory, pi_enc)
                    arm.init(factory)
                    piano_player.init(factory)

                    if input("Use Keyboard? (y/n)\n").lower() == "y":
                        run_hand_ssh_control()
                    else: 
                        print(f"[SERIAL] Starting Serial Receiver on {SERIAL_PORT}...")
                        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                        time.sleep(1)
                        ser.reset_input_buffer()
                        last_rx = time.time()
                        
                        while True:
                            chassis.ultrasonic_tick()
                            
                            if ser.in_waiting > 0:
                                line = ser.readline().decode('utf-8', errors='ignore').strip()
                                if line:
                                    if line.startswith("DATA:"):
                                        try:
                                            parts = line.split(":")[1].split(",")
                                            if len(parts) == 2:
                                                handle_hand_payload(int(parts[0]), int(parts[1]))
                                        except Exception: pass
                                    else:
                                        print(f"[ARDUINO MSG] {line}")
                                        
                            if time.time() - last_rx > 0.7: 
                                chassis.stop_drive()
                                print("[WARNING] Connection Lost. Halting...")
                                handle_hand_payload(0, 0) 
                                time.sleep(0.5) 
                            time.sleep(0.01)

                except KeyboardInterrupt: print("\n[INFO] Returning to main menu...")
                except Exception as e: print(f"[ERROR] HAND mode: {e}")
                finally:
                    chassis.stop_drive(); chassis.close_pins(); piano_player.close_pins()
                    arm.close_pins()
                    if 'ser' in locals() and ser.is_open: ser.close()
                    
            else: print("Invalid input.")

    except KeyboardInterrupt: print("\n[INFO] Interrupted by user.")
    finally:
        chassis.stop_drive()
        try: vision.shutdown()
        except Exception: pass 
        GPIO.cleanup()
        print("[SYSTEM] Safely powered down.")
