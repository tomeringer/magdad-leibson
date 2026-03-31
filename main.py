#!/usr/bin/env python3
import os, time, socket, math, sys, tty, termios, pigpio
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory

# Import as namespaces to avoid NoneType copy errors
import chassis
import gripper
import arm
import vision

os.environ['PIGPIO_ADDR'] = 'localhost'

# Constants
ARC_STOP_OFFSET_CM, Z0_STOP_DISTANCE_CM = 4.5, 21.0
ARC_MAX_OUTER_SPEED, ARC_SPEED_DIFF_FACTOR = 0.3, 1.06
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"
CALIBRATION_FILE_PATH = "Autonomy/stereo_calibration.pkl"

# Initial Setup
factory = PiGPIOFactory()
pi_enc = pigpio.pi()
GPIO.setmode(GPIO.BCM)

_arm_dir = 0
_prev_f = [0, 0, 0, 0]
_drive_hist = ["STOP", "STOP"]
_ignore_ultra = [False, None]
last_rx = 0.0


def get_average_distance():
    dist_l = abs(chassis.enc_left.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    dist_r = abs(chassis.enc_right.output_revolutions()) * chassis.WHEEL_CIRCUMFERENCE_CM
    return (dist_l + dist_l) / 2.0


def drive_arc(target_x, target_z):
    R = (target_x ** 2 + target_z ** 2) / (2 * abs(target_x))
    r_in, r_out = R - (chassis.TRACK_WIDTH_CM / 2), R + (chassis.TRACK_WIDTH_CM / 2)
    s_in = ARC_MAX_OUTER_SPEED * (r_in / r_out)
    v_l, v_r = (ARC_MAX_OUTER_SPEED, s_in) if target_x > 0 else (s_in, ARC_MAX_OUTER_SPEED)
    total_arc = R * math.atan2(target_z, R - abs(target_x)) - ARC_STOP_OFFSET_CM

    # Call through chassis namespace
    chassis.RIGHT_RPWM.value, chassis.LEFT_RPWM.value = v_r * ARC_SPEED_DIFF_FACTOR, v_l
    chassis.RIGHT_LPWM.value = chassis.LEFT_LPWM.value = 0.0
    chassis.enc_left.zero();
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
            time.sleep(0.3);
            gripper.move_step(1);
            return
        time.sleep(0.1)


def handle_payload(payload):
    global _arm_dir, _prev_f, _drive_hist, _ignore_ultra, last_rx
    f = [(payload >> (4 + i)) & 1 for i in range(4)]
    if all(f) and not all(_prev_f):
        chassis.stop_drive();
        arm.stop();
        bring_bottle_xz()
    else:
        if f[3] and not _prev_f[3]: gripper.move_step(1)
        if f[2] and not _prev_f[2]: gripper.move_step(0)
        _arm_dir = -1 if (f[0] and not f[1]) else 1 if (f[1] and not f[0]) else 0

        p, r = payload & 0x03, (payload >> 2) & 0x03
        req = "REV" if p == 1 else "FWD" if p == 2 else "LEFT" if r == 1 else "RIGHT" if r == 2 else "STOP"

        too_close = chassis.obstacle_too_close()
        if _ignore_ultra[0] and (req in ("STOP", "REV") or req != _ignore_ultra[1]): _ignore_ultra[0] = False
        if (not _ignore_ultra[0]) and req in {"FWD", "LEFT", "RIGHT"}:
            if _drive_hist[1] == "STOP" and _drive_hist[0] == req: _ignore_ultra[:] = [True, req]

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
        _drive_hist = [_drive_hist[1], req]
    _prev_f = f;
    last_rx = time.time()


def run_ssh_control():
    def getch():
        fd = sys.stdin.fileno();
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


if __name__ == "__main__":
    try:
        chassis.init(factory, pi_enc)
        gripper.init(factory);
        arm.init(factory);
        vision.init(LEFT_CAM_PATH, RIGHT_CAM_PATH, CALIBRATION_FILE_PATH)
        if input("Use Keyboard? (y/n)\n").lower() == "y":
            run_ssh_control()
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
            sock.bind(("0.0.0.0", 4210));
            sock.settimeout(0.02)
            while True:
                chassis.ultrasonic_tick()
                if _arm_dir != 0:
                    arm.run(_arm_dir == 1)
                else:
                    arm.stop()
                try:
                    data, _ = sock.recvfrom(1024)
                    if len(data) >= 3 and data[0] == 0xAA and data[2] == 0x55: handle_payload(data[1])
                except socket.timeout:
                    if time.time() - last_rx > 0.7: chassis.stop_drive()
    finally:
        chassis.stop_drive();
        vision.shutdown();
        GPIO.cleanup()