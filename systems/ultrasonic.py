import time
from typing import Optional

import RPi.GPIO as GPIO


# ============================================================
# ULTRASONIC
# ============================================================
TRIG = 5
ECHO = 6
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

ULTRA_ENABLED = True
ULTRA_STOP_CM = 40.0

ULTRA_MEASURE_PERIOD_SEC = 0.10   # measure at 10 Hz
ULTRA_PRINT_PERIOD_SEC = 0.50     # print at 2 Hz

_last_ultra_t = 0.0
_last_ultra_print_t = 0.0
last_distance_cm: Optional[float] = None

def measure_distance_cm() -> Optional[float]:
    """
    Returns distance in cm or None on timeout.
    """
    GPIO.output(TRIG, False)
    time.sleep(0.0002)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    t0 = time.time()
    while GPIO.input(ECHO) == 0:
        if time.time() - t0 > 0.03:
            return None

    pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        if time.time() - pulse_start > 0.03:
            return None

    pulse_end = time.time()
    return (pulse_end - pulse_start) * 17150.0


def tick() -> Optional[float]:
    """
    Periodically measures + periodically prints.
    Updates _last_distance_cm and returns it.
    """
    global _last_ultra_t, _last_ultra_print_t, last_distance_cm

    now = time.time()
    if not ULTRA_ENABLED:
        return last_distance_cm

    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        last_distance_cm = measure_distance_cm()

    if now - _last_ultra_print_t >= ULTRA_PRINT_PERIOD_SEC:
        _last_ultra_print_t = now
        if last_distance_cm is None:
            print("[ULTRA] dist=None (timeout)")
        else:
            print(f"[ULTRA] dist={last_distance_cm:.1f} cm")

    return last_distance_cm


def too_close() -> bool:
    return ULTRA_ENABLED and (last_distance_cm is not None) and (last_distance_cm < ULTRA_STOP_CM)
