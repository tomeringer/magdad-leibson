import time
import math
import RPi.GPIO as GPIO
from gpiozero import PWMOutputDevice

# Pins & Constants
US1_TRIG, US1_ECHO = 5, 6
US2_TRIG, US2_ECHO = 13, 19
RIGHT_RPWM_PIN, RIGHT_LPWM_PIN = 2, 3
LEFT_RPWM_PIN, LEFT_LPWM_PIN = 20, 21
ENC_LEFT_A, ENC_LEFT_B = 24, 25
ENC_RIGHT_A, ENC_RIGHT_B = 9, 10

ULTRA_STOP_CM = 40.0
US_TIMEOUT_SEC = 0.03
ULTRA_MEASURE_PERIOD_SEC = 0.10
SPEED_OF_SOUND_DIVISOR = 17150.0
WHEEL_CIRCUMFERENCE_CM = math.pi * 7.7
TRACK_WIDTH_CM = 39.0
SPEED_DIFF_FACTOR = 1.0

# Module State
_last_ultra_t = 0.0
_last_distances = [None, None]
RIGHT_RPWM = LEFT_RPWM = RIGHT_LPWM = LEFT_LPWM = None
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

    def output_revolutions(self): return self.counts / self.ppr_output

    def zero(self): self.counts = 0


def init(factory, pi_enc):
    global RIGHT_RPWM, LEFT_RPWM, RIGHT_LPWM, LEFT_LPWM, enc_left, enc_right
    GPIO.setup([US1_TRIG, US2_TRIG], GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup([US1_ECHO, US2_ECHO], GPIO.IN)
    RIGHT_RPWM = PWMOutputDevice(RIGHT_RPWM_PIN, frequency=1000, pin_factory=factory)
    RIGHT_LPWM = PWMOutputDevice(RIGHT_LPWM_PIN, frequency=1000, pin_factory=factory)
    LEFT_RPWM = PWMOutputDevice(LEFT_RPWM_PIN, frequency=1000, pin_factory=factory)
    LEFT_LPWM = PWMOutputDevice(LEFT_LPWM_PIN, frequency=1000, pin_factory=factory)
    enc_left = YellowJacketEncoder(pi_enc, ENC_LEFT_A, ENC_LEFT_B)
    enc_right = YellowJacketEncoder(pi_enc, ENC_RIGHT_A, ENC_RIGHT_B)


def ultrasonic_tick():
    global _last_ultra_t, _last_distances
    now = time.time()
    if now - _last_ultra_t >= ULTRA_MEASURE_PERIOD_SEC:
        _last_ultra_t = now
        GPIO.output(US1_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(US1_TRIG, False)
        t0 = time.time()
        while GPIO.input(US1_ECHO) == 0 and time.time() - t0 < US_TIMEOUT_SEC: pass
        ps = time.time()
        while GPIO.input(US1_ECHO) == 1 and time.time() - ps < US_TIMEOUT_SEC: pass
        d1 = (time.time() - ps) * SPEED_OF_SOUND_DIVISOR if time.time() - ps < US_TIMEOUT_SEC else None

        GPIO.output(US2_TRIG, True)
        time.sleep(0.00001)
        GPIO.output(US2_TRIG, False)
        t0 = time.time()
        while GPIO.input(US2_ECHO) == 0 and time.time() - t0 < US_TIMEOUT_SEC: pass
        ps = time.time()
        while GPIO.input(US2_ECHO) == 1 and time.time() - ps < US_TIMEOUT_SEC: pass
        d2 = (time.time() - ps) * SPEED_OF_SOUND_DIVISOR if time.time() - ps < US_TIMEOUT_SEC else None
        _last_distances = [d1, d2]


def obstacle_too_close():
    return any(d is not None and d < ULTRA_STOP_CM for d in _last_distances)


def stop_drive():
    RIGHT_RPWM.value = RIGHT_LPWM.value = LEFT_RPWM.value = LEFT_LPWM.value = 0.0


def drive_forward(speed=0.3):
    RIGHT_RPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value = LEFT_LPWM.value = 0.0


def drive_reverse(speed=0.3):
    RIGHT_LPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value = LEFT_RPWM.value = 0.0


def turn_right(speed=0.2):
    RIGHT_LPWM.value, LEFT_RPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_RPWM.value = LEFT_LPWM.value = 0.0


def turn_left(speed=0.2):
    RIGHT_RPWM.value, LEFT_LPWM.value = speed * SPEED_DIFF_FACTOR, speed
    RIGHT_LPWM.value = LEFT_RPWM.value = 0.0


def close_pins():
    RIGHT_RPWM.close()
    LEFT_RPWM.close()
    RIGHT_LPWM.close()
    LEFT_LPWM.close()