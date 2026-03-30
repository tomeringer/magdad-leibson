#!/usr/bin/env python3
import pigpio
import socket
import sys
import termios
import tty

import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

from arm import *
from chassis import *
# Import all from modules (simulating flat code)
from gripper import *
from vision import *

os.environ['PIGPIO_ADDR'] = 'localhost'

# --- Config ---
UDP_LISTEN_IP, UDP_PORT = "0.0.0.0", 4210
FRAME_START, FRAME_END = 0xAA, 0x55
UDP_TIMEOUT_SEC, SILENCE_STOP_SEC = 0.02, 0.7
CONTROL_PERIOD_SEC = 0.01
BOTTLE_SEARCH_TIMEOUT_SEC = 5.0
ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
CALIBRATION_FILE_PATH = "Autonomy/stereo_calibration.pkl"

# --- Global State ---
factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

last_rx = 0.0
_arm_dir = 0
_prev_f0 = _prev_f1 = _prev_f2 = _prev_f3 = 0
_prev_drive_req = _prev_prev_drive_req = "STOP"
_ignore_ultra_active = False
_ignore_ultra_cmd = None


def get_average_distance():
    dist_l = abs(enc_left.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM
    dist_r = abs(enc_right.output_revolutions()) * WHEEL_CIRCUMFERENCE_CM
    return (dist_l + dist_r) / 2.0


def drive_arc(target_x, target_z):
    R = (target_x ** 2 + target_z ** 2) / (2 * abs(target_x))
    r_in, r_out = R - (TRACK_WIDTH_CM / 2), R + (TRACK_WIDTH_CM / 2)
    s_in = ARC_MAX_OUTER_SPEED * (r_in / r_out)

    if target_x > 0:
        v_l, v_r = ARC_MAX_OUTER_SPEED, s_in
    else:
        v_l, v_r = s_in, ARC_MAX_OUTER_SPEED

    angle = math.atan2(target_z, R - abs(target_x))
    total_arc = R * angle - ARC_STOP_OFFSET_CM

    RIGHT_RPWM.value, LEFT_RPWM.value = v_r * ARC_SPEED_DIFF_FACTOR, v_l
    RIGHT_LPWM.value = LEFT_LPWM.value = 0.0

    enc_left.zero();
    enc_right.zero()
    while get_average_distance() < total_arc:
        time.sleep(0.01)
    stop_drive()


def bring_bottle_xz():
    servo_move_step(0)
    t0 = time.perf_counter()
    while True:
        res = detect_bottle_once()
        if time.perf_counter() - t0 > BOTTLE_SEARCH_TIMEOUT_SEC: return
        if res["found"]:
            drive_arc(res['X'], res['Z'])
            time.sleep(0.3)
            servo_move_step(1)
            break
        time.sleep(0.1)


def handle_payload(payload):
    global _prev_f0, _prev_f1, _prev_f2, _prev_f3, _arm_dir
    global _prev_drive_req, _prev_prev_drive_req, _ignore_ultra_active, _ignore_ultra_cmd

    flex = (payload >> 4) & 0x0F
    f0, f1, f2, f3 = (flex >> 0) & 1, (flex >> 1) & 1, (flex >> 2) & 1, (flex >> 3) & 1

    if (f0 == f1 == f2 == f3 == 1) and not (_prev_f0 == _prev_f1 == _prev_f2 == _prev_f3 == 1):
        stop_drive();
        stop_arm();
        bring_bottle_xz()
        _prev_f0, _prev_f1, _prev_f2, _prev_f3 = f0, f1, f2, f3
        return

    if f3 == 1 and _prev_f3 == 0: servo_move_step(1)
    if f2 == 1 and _prev_f2 == 0: servo_move_step(0)

    if f0 == 1 and f1 == 0:
        _arm_dir = -1
    elif f1 == 1 and f0 == 0:
        _arm_dir = +1
    else:
        _arm_dir = 0

    _prev_f0, _prev_f1, _prev_f2, _prev_f3 = f0, f1, f2, f3
    p_code, r_code = payload & 0x03, (payload >> 2) & 0x03
    drive_req = "REV" if p_code == 1 else "FWD" if p_code == 2 else "LEFT" if r_code == 1 else "RIGHT" if r_code == 2 else "STOP"

    too_close = obstacle_too_close()
    if _ignore_ultra_active and (drive_req in ("STOP", "REV") or drive_req != _ignore_ultra_cmd):
        _ignore_ultra_active = False
    if (not _ignore_ultra_active) and (drive_req in {"FWD", "LEFT", "RIGHT"}):
        if _prev_drive_req == "STOP" and _prev_prev_drive_req == drive_req:
            _ignore_ultra_active, _ignore_ultra_cmd = True, drive_req

    ignore = (_ignore_ultra_active and drive_req == _ignore_ultra_cmd)
    if drive_req == "REV":
        drive_reverse()
    elif drive_req == "FWD":
        drive_forward() if (not too_close or ignore) else stop_drive()
    elif drive_req == "LEFT":
        turn_left() if (not too_close or ignore) else stop_drive()
    elif drive_req == "RIGHT":
        turn_right() if (not too_close or ignore) else stop_drive()
    else:
        stop_drive()

    _prev_prev_drive_req, _prev_drive_req = _prev_drive_req, drive_req


def run_glove_loop():
    global last_rx
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_PORT))
    sock.settimeout(UDP_TIMEOUT_SEC)
    last_rx = time.time()
    while True:
        ultrasonic_tick()
        if _arm_dir != 0:
            run_arm(_arm_dir == 1)
        else:
            stop_arm()
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 3 and data[0] == FRAME_START and data[2] == FRAME_END:
                handle_payload(data[1])
                last_rx = time.time()
        except socket.timeout:
            if time.time() - last_rx > SILENCE_STOP_SEC: stop_drive()
        time.sleep(CONTROL_PERIOD_SEC)


def run_ssh_control():
    def getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    while True:
        ultrasonic_tick()
        c = getch().lower()
        if c == 'w':
            drive_forward()
        elif c == 's':
            drive_reverse()
        elif c == 'a':
            turn_left()
        elif c == 'd':
            turn_right()
        elif c == 'r':
            run_arm(True)
        elif c == 'f':
            run_arm(False)
        elif c == 'o':
            servo_move_step(0)
        elif c == 'c':
            servo_move_step(1)
        elif c == 'z':
            bring_bottle_xz()
        elif c == ' ' or c == 'k':
            stop_drive();
            stop_arm()
        elif c == 'q':
            break
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        init_vision(LEFT_CAM_PATH, RIGHT_CAM_PATH, CALIBRATION_FILE_PATH)
        init_gripper(factory)
        init_arm(factory)
        init_chassis(factory, pi_enc)
        mode = input("Use Keyboard? (y/n)\n").lower()
        if mode == "y":
            run_ssh_control()
        else:
            run_glove_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop_drive();
        shutdown_vision();
        GPIO.cleanup()
