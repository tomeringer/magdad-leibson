from gpiozero import PWMOutputDevice
from enum import Enum


# ============================================================
# DC MOTORS (IBT-2) -- PWM + trim
# ============================================================
PWM_HZ = 1000

# M1 (right motor in your current mapping)
M1_RPWM = PWMOutputDevice(18, frequency=PWM_HZ, initial_value=0)
M1_LPWM = PWMOutputDevice(23, frequency=PWM_HZ, initial_value=0)

# M2 (LEFT motor: physical pins 18+22 -> BCM 24+25)
M2_RPWM = PWMOutputDevice(24, frequency=PWM_HZ, initial_value=0)
M2_LPWM = PWMOutputDevice(25, frequency=PWM_HZ, initial_value=0)

# Tune this to remove drift (left motor slower => < 1.0)
M1_SCALE = 1.00
M2_SCALE = 0.90


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def motor_stop():
    M1_RPWM.value = 0.0
    M1_LPWM.value = 0.0
    M2_RPWM.value = 0.0
    M2_LPWM.value = 0.0


def motor1_forward(speed: float = 1.0):
    s = _clamp01(speed) * M1_SCALE
    M1_RPWM.value = s
    M1_LPWM.value = 0.0


def motor1_reverse(speed: float = 1.0):
    s = _clamp01(speed) * M1_SCALE
    M1_RPWM.value = 0.0
    M1_LPWM.value = s


def motor2_forward(speed: float = 1.0):
    s = _clamp01(speed) * M2_SCALE
    M2_RPWM.value = s
    M2_LPWM.value = 0.0


def motor2_reverse(speed: float = 1.0):
    s = _clamp01(speed) * M2_SCALE
    M2_RPWM.value = 0.0
    M2_LPWM.value = s

def run_desired(desired):
    if desired == States.FORWARD:
        motor1_forward()
        motor2_forward()
    elif desired == States.REVERSE:
        motor1_reverse()
        motor2_reverse()
    elif desired == States.TURN_RIGHT:
        motor1_forward()
        motor2_reverse()
    elif desired == States.TURN_LEFT:
        motor1_reverse()
        motor2_forward()
    else:
        motor_stop()


class States(Enum):
    FORWARD = 1
    REVERSE = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    STOP = 5