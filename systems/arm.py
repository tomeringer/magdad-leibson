import time
from gpiozero import OutputDevice


class Arm:
    def __init__(self):
        # ============================================================
        # STEPPER (L298N IN1..IN4)
        # ============================================================
        self.STEP_IN1 = 23
        self.STEP_IN2 = 22
        self.STEP_IN3 = 27
        self.STEP_IN4 = 17

        self.IN1 = OutputDevice(self.STEP_IN1, active_high=True, initial_value=False)
        self.IN2 = OutputDevice(self.STEP_IN2, active_high=True, initial_value=False)
        self.IN3 = OutputDevice(self.STEP_IN3, active_high=True, initial_value=False)
        self.IN4 = OutputDevice(self.STEP_IN4, active_high=True, initial_value=False)

        self.STEPPER_STEP_DELAY = 0.002   # L298N is slower than A4988; 0.001..0.005 typical
        self.STEPPER_STEP_CHUNK = 20

        # Half-step sequence (8 states) for a 4-wire bipolar stepper via H-bridges
        # If direction is wrong or it "buzzes", we can reverse this or swap coil order.
        self.HALF_STEP_SEQ = [
            (1, 0, 0, 0),
            (1, 0, 1, 0),
            (0, 0, 1, 0),
            (0, 1, 1, 0),
            (0, 1, 0, 0),
            (0, 1, 0, 1),
            (0, 0, 0, 1),
            (1, 0, 0, 1),
        ]

    def _stepper_set(self, a, b, c, d):
        self.IN1.value = 1 if a else 0
        self.IN2.value = 1 if b else 0
        self.IN3.value = 1 if c else 0
        self.IN4.value = 1 if d else 0

    def stepper_release(self):
        self._stepper_set(0, 0, 0, 0)

    def stepper_move(self, steps: int, direction: int):
        """
        steps: number of half-steps to execute
        direction: 1 forward, 0 reverse
        """
        seq = self.HALF_STEP_SEQ if direction else list(reversed(self.HALF_STEP_SEQ))
        for i in range(abs(steps)):
            a, b, c, d = seq[i % len(seq)]
            self._stepper_set(a, b, c, d)
            time.sleep(self.STEPPER_STEP_DELAY)
        self.stepper_release()
