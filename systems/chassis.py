from enum import Enum

from gpiozero import PWMOutputDevice


class Chassis:
    def __init__(self):
        # ============================================================
        # DC MOTORS (IBT-2) -- PWM + trim
        # ============================================================
        self.PWM_HZ = 1000

        # M1 (right motor in your current mapping)
        self.M1_RPWM = PWMOutputDevice(2, frequency=self.PWM_HZ, initial_value=0)
        self.M2_RPWM = PWMOutputDevice(3, frequency=self.PWM_HZ, initial_value=0)

        # M2 (LEFT motor: physical pins 18+22 -> BCM 24+25)
        self.M1_LPWM = PWMOutputDevice(20, frequency=self.PWM_HZ, initial_value=0)
        self.M2_LPWM = PWMOutputDevice(21, frequency=self.PWM_HZ, initial_value=0)

        # Tune this to remove drift (left motor slower => < 1.0)
        self.M1_SCALE = 1.00
        self.M2_SCALE = 0.90

    def _clamp01(self, x: float) -> float:
        return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

    def motor_stop(self):
        self.M1_RPWM.value = 0.0
        self.M1_LPWM.value = 0.0
        self.M2_RPWM.value = 0.0
        self.M2_LPWM.value = 0.0

    def motor1_forward(self, speed: float = 1.0):
        s = self._clamp01(speed) * self.M1_SCALE
        self.M1_RPWM.value = s
        self.M1_LPWM.value = 0.0

    def motor1_reverse(self, speed: float = 1.0):
        s = self._clamp01(speed) * self.M1_SCALE
        self.M1_RPWM.value = 0.0
        self.M1_LPWM.value = s

    def motor2_forward(self, speed: float = 1.0):
        s = self._clamp01(speed) * self.M2_SCALE
        self.M2_RPWM.value = s
        self.M2_LPWM.value = 0.0

    def motor2_reverse(self, speed: float = 1.0):
        s = self._clamp01(speed) * self.M2_SCALE
        self.M2_RPWM.value = 0.0
        self.M2_LPWM.value = s

    def run_desired(self, desired):
        if desired == States.FORWARD:
            self.motor1_forward()
            self.motor2_forward()
        elif desired == States.REVERSE:
            self.motor1_reverse()
            self.motor2_reverse()
        elif desired == States.TURN_RIGHT:
            self.motor1_forward()
            self.motor2_reverse()
        elif desired == States.TURN_LEFT:
            self.motor1_reverse()
            self.motor2_forward()
        else:
            self.motor_stop()


class States(Enum):
    FORWARD = 1
    REVERSE = 2
    TURN_RIGHT = 3
    TURN_LEFT = 4
    STOP = 5
