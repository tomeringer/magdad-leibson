#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
os.environ['PIGPIO_ADDR'] = 'localhost'

import time
import math
import pickle
import threading
import cv2
import numpy as np
from flask import Flask, Response
import pigpio
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice, Servo
from gpiozero.pins.pigpio import PiGPIOFactory
from ultralytics import YOLO

# ============================================================
# CONFIGURATION
# ============================================================
RED_LED_PIN = 26
GREEN_LED_PIN = 16

factory = PiGPIOFactory()
    
# ---- Stereo / YOLO config ----
CALIBRATION_FILE_PATH = r"Autonomy/stereo_calibration.pkl"
LEFT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-video-index0"
RIGHT_CAM_PATH = "/dev/v4l/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1:1.0-video-index0"

W = 640
H = 480
CAM_FPS = 5
DEFAULT_IMG_SIZE = (W, H)

BOTTLE_CLASS_ID = 39
YOLO_CONF = 0.50
MATCH_Y_TOL = 10

STEPPER_STEP_DELAY_SEC = 0.0015

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(RED_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(GREEN_LED_PIN, GPIO.OUT, initial=GPIO.HIGH)

# ============================================================
# GLOBALS FOR THREAD COMMUNICATION
# ============================================================
# These globals allow the background vision thread to share data
# safely with the Flask server and the Robot control loop.
_global_stream_frame = np.zeros((480, 1280, 3), dtype=np.uint8)
_global_bottle_data = {"found": False, "X": 0.0, "Y": 0.0, "Z": 0.0, "cx": 0, "cy": 0}
_vision_running = True

# ============================================================
# FLASK WEB SERVER SETUP
# ============================================================
app = Flask(__name__)

def generate_stream():
    """Yields the latest combined annotated frame to the web browser."""
    global _global_stream_frame
    while True:
        # Encode the global frame to JPEG
        ret, buffer = cv2.imencode('.jpg', _global_stream_frame)
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        # Small sleep to prevent CPU hogging
        time.sleep(0.05)

@app.route('/')
def index():
    html_content = """
    <html>
        <head><title>Pi Autonomy Vision</title></head>
        <body style="text-align: center; background-color: #222; color: white; font-family: sans-serif;">
            <h2>Stereo Vision & Autonomy Status</h2>
            <img src="/video_feed" width="1280" height="480" style="border: 2px solid white; max-width: 100%;"/>
        </body>
    </html>
    """
    return html_content

@app.route('/video_feed')
def video_feed():
    return Response(generate_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ============================================================
# SERVO & MOTORS
# ============================================================
SERVO_PIN = 12
servo = Servo(
    SERVO_PIN,
    initial_value=None,
    min_pulse_width=0.5 / 1000,
    max_pulse_width=2.5 / 1000,
    pin_factory=factory
)

MOVE_STEP = 0.39
current_pos = 0.0
servo_is_open = True

def servo_move_step(direction):
    global current_pos, servo_is_open
    want_close = (direction == 1)
    if want_close and (not servo_is_open): return
    if (not want_close) and servo_is_open: return

    new_val = current_pos + MOVE_STEP if want_close else current_pos - MOVE_STEP
    new_val = max(-1.0, min(1.0, new_val))
    current_pos = new_val
    servo.value = current_pos
    servo_is_open = (not want_close)

RIGHT_RPWM = PWMOutputDevice(2, frequency=1000, initial_value=0, pin_factory=factory)
RIGHT_LPWM = PWMOutputDevice(3, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_RPWM = PWMOutputDevice(20, frequency=1000, initial_value=0, pin_factory=factory)
LEFT_LPWM = PWMOutputDevice(21, frequency=1000, initial_value=0, pin_factory=factory)

def stop_drive():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0

def drive_forward(speed: float = 0.45):
    RIGHT_RPWM.value, LEFT_RPWM.value = speed + 0.04, speed
    RIGHT_LPWM.value, LEFT_LPWM.value = 0.0, 0.0

def drive_reverse(speed: float = 0.45):
    RIGHT_LPWM.value, LEFT_LPWM.value = speed + 0.04, speed
    RIGHT_RPWM.value, LEFT_RPWM.value = 0.0, 0.0

def turn_right(speed: float = 0.10):
    RIGHT_LPWM.value, LEFT_RPWM.value = speed + 0.04, speed
    RIGHT_RPWM.value, LEFT_LPWM.value = 0.0, 0.0
    
def turn_left(speed: float = 0.10):
    RIGHT_RPWM.value, LEFT_LPWM.value = speed + 0.04, speed
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0
    RIGHT_RPWM.value, LEFT_LPWM.value = speed + 0.01, speed
    RIGHT_LPWM.value, LEFT_RPWM.value = 0.0, 0.0

# ============================================================
# ENCODER & STEPPER
# ============================================================
pi_enc = pigpio.pi()

class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1, 0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi, gpio_a, gpio_b):
        self.pi, self.gpio_a, self.gpio_b = pi, gpio_a, gpio_b
        self.ppr_output = 384.5 * (100 / 106)
        self.counts_per_rev_output = self.ppr_output
        self.counts = 0
        self._last_state = (pi.read(gpio_a) << 1) | pi.read(gpio_b)
        self.pi.set_mode(gpio_a, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_a, pigpio.PUD_UP)
        self.pi.set_mode(gpio_b, pigpio.INPUT)
        self.pi.set_pull_up_down(gpio_b, pigpio.PUD_UP)
        self._cba = pi.callback(gpio_a, pigpio.EITHER_EDGE, self._cb)
        self._cbb = pi.callback(gpio_b, pigpio.EITHER_EDGE, self._cb)
        self._t_last, self._c_last, self._output_rpm = time.time(), 0, 0.0

    def _cb(self, gpio, level, tick):
        new_state = (self.pi.read(self.gpio_a) << 1) | self.pi.read(self.gpio_b)
        self.counts += self._TRANS.get((self._last_state << 2) | new_state, 0)
        self._last_state = new_state

    def update(self, window_s=0.2):
        dt = time.time() - self._t_last
        if dt < window_s: return
        self._output_rpm = 60.0 * ((self.counts - self._c_last) / self.counts_per_rev_output) / dt
        self._t_last, self._c_last = time.time(), self.counts

    def output_revolutions(self): return self.counts / self.counts_per_rev_output

    def zero(self): self.counts = self._c_last = 0

enc_left = YellowJacketEncoder(pi_enc, 24, 25)
enc = YellowJacketEncoder(pi_enc, 4, 18)

class StepperMotor:
    def __init__(self, pins):
        self.pins, self.pi = pins, pigpio.pi()
        self.seq = [[1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]]
        for p in self.pins: self.pi.set_mode(p, pigpio.OUTPUT)

    def step_chunk(self, steps, direction=1, delay_sec=0.002):
        direction = 1 if direction >= 0 else -1
        for _ in range(int(steps)):
            for step in range(4):
                idx = step if direction == 1 else 3 - step
                for i, p in enumerate(self.pins): self.pi.write(p, self.seq[idx][i])
                time.sleep(delay_sec)
        self.deenergize()

    def deenergize(self):
        for p in self.pins: self.pi.write(p, 0)

    def close(self):
        self.deenergize(); self.pi.stop()

_stepper = StepperMotor([23, 22, 27, 17])

# ============================================================
# VISION & YOLO LOGIC
# ============================================================
_cap_l = _cap_r = _maps_l = _maps_r = _Q = _model = None

def load_and_prepare_calibration(file_path):
    try:
        with open(file_path, 'rb') as f:
            data = pickle.load(f)
        K1, D1 = data['cameraMatrix1'], data['distCoeffs1']
        K2, D2 = data['cameraMatrix2'], data['distCoeffs2']
        R, T, Q = data['R'], data['T'], data['Q']
    except Exception as e:
        raise RuntimeError(f"Calibration load failed: {e}")

    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(
        cameraMatrix1=K1, distCoeffs1=D1, cameraMatrix2=K2, distCoeffs2=D2,
        imageSize=DEFAULT_IMG_SIZE, R=R, T=T, alpha=-1
    )
    map1_l, map2_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    map1_r, map2_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, DEFAULT_IMG_SIZE, cv2.CV_32FC1)
    return (map1_l, map2_l), (map1_r, map2_r), Q

def calculate_3d_coords(disparity, x, y, Q_matrix):
    point = np.array([x, y, disparity, 1], dtype=np.float32)
    hp = np.dot(Q_matrix, point)
    w = hp[3]
    if w == 0: return (0, 0, 0)
    return (hp[0] / w, hp[1] / w, hp[2] / w)

def vision_background_worker():
    """ 
    This thread continuously reads cameras, runs YOLO, draws bounding boxes 
    and updates the global frame for the Flask server to broadcast.
    """
    global _global_stream_frame, _global_bottle_data, _vision_running
    blank_frame = np.zeros((H, W, 3), dtype=np.uint8)

    while _vision_running:
        # Read from cameras
        success_l, frame_l_orig = _cap_l.read()
        success_r, frame_r_orig = _cap_r.read()

        # Handle hardware timeouts securely
        if not success_l or frame_l_orig is None:
            frame_l_orig = blank_frame.copy()
            cv2.putText(frame_l_orig, "LEFT: NO SIGNAL", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        if not success_r or frame_r_orig is None:
            frame_r_orig = blank_frame.copy()
            cv2.putText(frame_r_orig, "RIGHT: NO SIGNAL", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        # Skip YOLO if we lost cameras
        if not success_l or not success_r:
            _global_stream_frame = np.hstack((frame_l_orig, frame_r_orig))
            continue

        # Rectify frames
        frame_l = cv2.remap(frame_l_orig, _maps_l[0], _maps_l[1], cv2.INTER_LINEAR)
        frame_r = cv2.remap(frame_r_orig, _maps_r[0], _maps_r[1], cv2.INTER_LINEAR)

        # YOLO inference
        results_l = _model.predict(frame_l, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)
        results_r = _model.predict(frame_r, conf=YOLO_CONF, classes=[BOTTLE_CLASS_ID], verbose=False)

        X = Y = Z = None
        cx = cy = disparity = None
        found_match = False

        # Check left camera for bottle
        if results_l and len(results_l[0].boxes) > 0:
            box_l = results_l[0].boxes[0]
            cx = int(box_l.xywh[0][0].item())
            cy = int(box_l.xywh[0][1].item())
            x1_l, y1_l, x2_l, y2_l = map(int, box_l.xyxy[0])

            closest_box_r = None
            if results_r and len(results_r[0].boxes) > 0:
                for box_r in results_r[0].boxes:
                    y_r = int(box_r.xywh[0][1].item())
                    if abs(cy - y_r) < MATCH_Y_TOL:
                        closest_box_r = box_r
                        break

            if closest_box_r is not None:
                x_r = int(closest_box_r.xywh[0][0].item())
                disparity = abs(cx - x_r)
                
                # Bounding box for right frame
                x1_r, y1_r, x2_r, y2_r = map(int, closest_box_r.xyxy[0])
                cv2.rectangle(frame_r, (x1_r, y1_r), (x2_r, y2_r), (255, 0, 0), 2)
                cv2.putText(frame_r, "Matched", (x1_r, y1_r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                if disparity > 0:
                    X_raw, Y_raw, Z_raw = calculate_3d_coords(disparity, cx, cy, _Q)
                    
                    # Apply specific geometry corrections
                    alpha = math.atan2(X_raw, Z_raw)
                    r = math.hypot(X_raw, Z_raw)
                    alpha = alpha + math.radians(3)
                    X = math.sin(alpha) * r - 5 
                    Y = Y_raw
                    Z = math.cos(alpha) * r
                    found_match = True

                    # Draw bounding box and accurate coordinates on left frame
                    cv2.rectangle(frame_l, (x1_l, y1_l), (x2_l, y2_l), (0, 255, 0), 2)
                    coord_text = f"X: {X:.1f}  Y: {Y:.1f}  Z: {Z:.1f}"
                    # Background for text to make it readable
                    cv2.rectangle(frame_l, (x1_l, y1_l - 30), (x1_l + 250, y1_l), (0, 255, 0), -1)
                    cv2.putText(frame_l, coord_text, (x1_l + 5, y1_l - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
            else:
                # Bottle found in left but no match in right
                cv2.rectangle(frame_l, (x1_l, y1_l), (x2_l, y2_l), (0, 165, 255), 2)
                cv2.putText(frame_l, "No Match", (x1_l, y1_l - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        # Update global state for Robot Autonomy logic
        _global_bottle_data = {
            "found": found_match,
            "X": X, "Y": Y, "Z": Z,
            "cx": cx, "cy": cy
        }

        # Update global frame for Flask server
        _global_stream_frame = np.hstack((frame_l, frame_r))

def init_vision_and_threads():
    global _model, _cap_l, _cap_r, _maps_l, _maps_r, _Q
    
    print("[SYSTEM] Starting cameras and loading AI model...", flush=True)
    _model = YOLO('Autonomy/yolov8n_ncnn_model', task='detect')

    real_left = os.path.realpath(LEFT_CAM_PATH)
    real_right = os.path.realpath(RIGHT_CAM_PATH)

    _cap_l = cv2.VideoCapture(real_left, cv2.CAP_V4L2)
    _cap_r = cv2.VideoCapture(real_right, cv2.CAP_V4L2)

    for cap in [_cap_l, _cap_r]:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    _maps_l, _maps_r, _Q = load_and_prepare_calibration(CALIBRATION_FILE_PATH)

    # Start the continuous vision processing thread
    threading.Thread(target=vision_background_worker, daemon=True).start()
    
    # Start the Flask web server in a background thread
    # use_reloader=False is mandatory when running Flask in a thread
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False), daemon=True).start()
    
    print("[VISION] System fully initialized. Web server running on port 5000.", flush=True)

# ============================================================
# MOTION & AUTONOMY
# ============================================================
def drive_distance(d_cm, forward: bool):
    enc.zero()
    enc.update()
    d_cm = d_cm - 1
    if forward:
        drive_forward()
    else:
        drive_reverse()
    while abs(enc.output_revolutions()) * (math.pi * 7.2) < d_cm:
        time.sleep(0.01)
        enc.update()
    stop_drive()

def turn_angle(theta_rad, left_turn: bool):
    theta_rad = theta_rad - math.radians(5)
    enc.zero()
    enc.update()
    radius = 39.0 / 2.0
    segment = radius * abs(theta_rad)
    if left_turn:
        turn_left(0.4)
    else:
        turn_right(0.4)
    while abs(enc.output_revolutions()) * (math.pi * 7.2) < segment:
        time.sleep(0.01)
        enc.update()
    stop_drive()

def bring_bottle_xz():
    time.sleep(2)
    z0 = 22
    t0 = time.perf_counter()
    
    bottle_data = None
    while True:
        # Read directly from the continuously updated global data
        # No hardware reading needed here, the background thread handles it!
        now = time.perf_counter()
        if now - t0 > 10:
            break
            
        if _global_bottle_data["found"]:
            bottle_data = dict(_global_bottle_data) # Safe copy
            print(f"Target Acquired -> X={bottle_data['X']:.2f}, Z={bottle_data['Z']:.2f}")
            original_z = bottle_data['Z']
            new_z = z0 + original_z
            alpha = math.atan2(bottle_data['X'], new_z)
            break
        else:   
            print("Searching for bottle...")
            time.sleep(0.5)

    if bottle_data:
        turn_angle(alpha, alpha < 0)
        print("Completed turn towards bottle.")
        time.sleep(2)
        drive_distance(math.hypot(bottle_data["Z"], bottle_data["X"]), True)
        print("Arrived at bottle location.")
        time.sleep(1)
        servo_move_step(0)
    else:
        print("Operation timeout: Bottle not locked.")

def track_bottle_continuous():
    print("\n[VISION] Monitoring bottle. Look at the web stream! Press Ctrl+C to stop.")
    try:
        while True:
            if _global_bottle_data["found"]:
                print(f"Tracking -> X: {_global_bottle_data['X']:.1f}, Y: {_global_bottle_data['Y']:.1f}, Z: {_global_bottle_data['Z']:.1f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[VISION] Tracking stopped. Returning to menu.")

# ============================================================
# MAIN EXECUTION
# ============================================================
if __name__ == "__main__":
    try:
        init_vision_and_threads()
        
        # Give hardware 2 seconds to warm up and fetch the first frame
        time.sleep(2) 
        
        print("\n--- ROBOT SYSTEM READY ---")
        print("Live view available at http://<Raspberry-Pi-IP>:5000")
        print("Type 'c' and press Enter to start bringing the bottle.")
        print("Type 'b' to view continuous tracking data.")
        print("Type 'q' and press Enter to quit.\n")
        
        while True:
            cmd = input("Command (c/b/q): ").strip().lower()
            if cmd == 'c':
                bring_bottle_xz()
            elif cmd == 'b':
                track_bottle_continuous()
            elif cmd == 'q':
                break
                
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
    finally:
        _vision_running = False
        stop_drive()
        GPIO.output(RED_LED_PIN, GPIO.HIGH)
        GPIO.output(GREEN_LED_PIN, GPIO.HIGH)
        if _cap_l is not None: _cap_l.release()
        if _cap_r is not None: _cap_r.release()
        GPIO.cleanup()
        print("[OFF] System Stopped.")