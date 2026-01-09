import RPi.GPIO as GPIO
import time
import math

# ================= GPIO SETUP =================
GPIO.setmode(GPIO.BCM)

# Ultrasonic 1
TRIG_1 = 5
ECHO_1 = 6

# Ultrasonic 2
TRIG_2 = 12
ECHO_2 = 13

# ### NEW ### LED
LED_PIN = 17   # physical pin 11

GPIO.setup(TRIG_1, GPIO.OUT)
GPIO.setup(ECHO_1, GPIO.IN)
GPIO.setup(TRIG_2, GPIO.OUT)
GPIO.setup(ECHO_2, GPIO.IN)

# ### NEW ###
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.output(LED_PIN, GPIO.LOW)

GPIO.output(TRIG_1, False)
GPIO.output(TRIG_2, False)

time.sleep(0.5)

# ================= CONSTANT =================
BASELINE_D = 37.5  # cm

# ================= MEASUREMENT =================
def measure_distance(trig, echo, timeout=0.02):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()

    while GPIO.input(echo) == 0:
        if time.time() - start_time > timeout:
            return None
    pulse_start = time.time()

    while GPIO.input(echo) == 1:
        if time.time() - pulse_start > timeout:
            return None
    pulse_end = time.time()

    duration = pulse_end - pulse_start
    return (duration * 34300) / 2  # cm

# ================= GEOMETRY =================
def compute_height(L1, L2, D):
    if abs(L1 - L2) >= D or L1 + L2 <= D:
        return None

    s = (L1 + L2 + D) / 2
    area = math.sqrt(
        s * (s - L1) * (s - L2) * (s - D)
    )
    return (2 * area) / D

def compute_horizontal_offset(L1, L2, D):
    return (L1**2 - L2**2) / (2 * D)

# ================= DRIVING CONDITION =================
def can_drive(h, x):
    if h is None or x is None:
        return False
    if h < 45:
        return False
    return True

# ================= MAIN LOOP =================
try:
    while True:
        L1 = measure_distance(TRIG_1, ECHO_1)
        time.sleep(0.06)
        L2 = measure_distance(TRIG_2, ECHO_2)

        if L1 is not None and L2 is not None:
            h = compute_height(L1, L2, BASELINE_D)
            x = compute_horizontal_offset(L1, L2, BASELINE_D)
        else:
            h = None
            x = None

        drive = can_drive(h, x)

        # ### NEW ### LED control
        if drive:
            GPIO.output(LED_PIN, GPIO.LOW)   # OK to drive
        else:
            GPIO.output(LED_PIN, GPIO.HIGH)  # STOP

        # -------- Pretty printing --------
        print("===================================")
        print(f"US1 distance: {L1:6.1f} cm" if L1 else "US1 distance: timeout")
        print(f"US2 distance: {L2:6.1f} cm" if L2 else "US2 distance: timeout")

        if h is not None:
            print(f"Forward distance (h):     {h:6.1f} cm")
            print(f"Horizontal offset (x):    {x:6.1f} cm")
        else:
            print("Geometry: INVALID")

        print("DRIVE:", "YES" if drive else "NO")
        print("LED:", "ON (STOP)" if not drive else "OFF")
        print("===================================\n")

        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nStopping...")
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
