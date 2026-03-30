import time
import math
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice

# Constants
US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19
RIGHT_RPWM_PIN = 2
RIGHT_LPWM_PIN = 3
LEFT_RPWM_PIN = 20
LEFT_LPWM_PIN = 21
ENC_LEFT_A, ENC_LEFT_B = 24, 25
ENC_RIGHT_A, ENC_RIGHT_B = 9, 10

ULTRA_STOP_CM = 40.0
US_TIMEOUT_SEC = 0.03
ULTRA_MEASURE_PERIOD_SEC = 0.10
SPEED_OF_SOUND_DIVISOR = 17150.0
WHEEL_CIRCUMFERENCE_CM = math.pi * 7.2
TRACK_WIDTH_CM = 39.0
SPEED_DIFF_FACTOR = 1.04
DEFAULT_DRIVE_SPEED = 0.3
DEFAULT_REVERSE_SPEED = 0.3
DEFAULT_TURN_SPEED = 0.2

# Module State
_last_ultra_t = 0.0
_last_distances = [None, None]
RIGHT_RPWM = RIGHT_LPWM = LEFT_RPWM = LEFT_LPWM = None
enc_left = enc_right = None


class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
              0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi, gpio_a, gpio_b):
        self.pi, self.gpio_a, self.gpio_b = pi, gpio_a, gpio_b
        self.ppr_output = 384.5 * (100 / 106)
        self.counts = 0
        self._last_state = (pi.read(gpio_a) << 1) | pi.read(gpio_b)
        self._cba = pi.callback(gpio_a, 3, self._cb)
        self._cbb = pi.callback(gpio_b, 3, self._cb)

    def _cb(self, gpio, level, tick):
        new_state = (self.pi.read(self.gpio_a) << 1) | self.pi.read(self.gpio_b)
        self.counts += self._TRANS.get((self._last_state << 2) | new_state, 0)
        self._last_state = new_state

    def update(self): pass  # Logic was empty/minimal in original update for arc

    def output_revolutions(self): return self.counts / self.ppr_output

    def zero(self): self.counts = 0


def init_chassis(factory, pi_enc):
    global RIGHT_RPWM, RIGHT_LPWM, LEFT_RPWM, LEFT_LPWM, enc_left, enc_right
    GPIO.setup([US1_TRIG, US2_TRIG], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([US1_ECHO, US2_ECHO], GPIO.IN)

    RIGHT_RPWM = PWMOutputDevice(RIGHT_RPWM_PIN, frequency=1000, pin_factory=factory)
    RIGHT_LPWM = PWMOutputDevice(RIGHT_LPWM_PIN, frequency=1000, pin_factory=factory)
    LEFT_RPWM = PWMOutputDevice(LEFT_RPWM_PIN, frequency=1000, pin_factory=factory)
    LEFT_LPWM = PWMOutputDevice(LEFT_LPWM_PIN, frequency=1000, pin_factory=factory)

    enc_left = YellowJacketEncoder(pi_enc, ENC_LEFT_A, ENC_LEFT_B)
    enc_right = YellowJacketEncoder(pi_enc, ENC_RIGHT_A, ENC_RIGHT_B)


def _measure_distance_cm(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    t0 = time.time()
    while GPIO.input(echo) == 0:
        if time.time() - t0 > US_TIMEOUT_SEC: return None
    ps = time.time()
    while GPIO.input(echo) == 1:
        if time.time() - ps > US_TIMEOUT_SEC: return None
    return (time.time() - ps) * SPEED_OF_SOUND_DIVISOR


def ultrasonic_tick():
    global _last_ultra_t, _last_distances
    now = time.time()
    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        _last_distances = [_measure_distance_cm(US1_TRIG, US1_ECHO), _measure_distance_cm(US2_TRIG, US2_ECHO)]


def obstacle_too_close():
    return any(d is not None and d < ULTRA_STOP_CM for d in _last_distances)


def stop_drive():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


def drive_forward(speed=DEFAULT_DRIVE_SPEED):
    RIGHT_RPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value = LEFT_LPWM.value = 0.0


def drive_reverse(speed=DEFAULT_REVERSE_SPEED):
    RIGHT_LPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value = LEFT_RPWM.value = 0.0


def turn_right(speed=DEFAULT_TURN_SPEED):
    RIGHT_LPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value = LEFT_LPWM.value = 0.0


def turn_left(speed=DEFAULT_TURN_SPEED):
    RIGHT_RPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value = LEFT_RPWM.value = 0.0