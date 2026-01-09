import time

import RPi.GPIO as GPIO


# ============================================================
# STEPPER (L298N 4-wire, your working implementation)
# ============================================================
class StepperMotor:
    def __init__(self, name, pins):
        self.name = name
        self.pins = pins

        # Full-step sequence (4 states)
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1],  # slight variant sometimes helps; keep if your wiring likes it
        ]

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

    def move(self, steps, direction=1, speed=0.005):
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")

        try:
            for _ in range(int(steps)):
                for step in range(len(self.seq)):
                    step_index = step if direction == 1 else (len(self.seq) - 1 - step)
                    pattern = self.seq[step_index]
                    for i, pin in enumerate(self.pins):
                        GPIO.output(pin, pattern[i])
                    time.sleep(speed)
        finally:
            self.stop()

    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, 0)


class Arm:
    def __init__(self):
        # Used already: 5,6,12,18,23,24,25
        self.STEPPER_PINS = [16, 19, 20, 21]  # L298N IN1..IN4
        self._stepper = StepperMotor("Stepper(L298N)", self.STEPPER_PINS)

        self.STEPPER_SPEED_SEC = 0.005
        self.STEPPER_STEP_CHUNK = 50

    def stepper_move(self, steps: int, direction: int):
        """
        direction: 1 = forward, 0 = reverse
        steps: number of step-cycles
        """
        if steps is None or direction is None:
            self.stop()
            return

        dir_pm = 1 if direction else -1
        self._stepper.move(int(abs(steps)), direction=dir_pm, speed=self.STEPPER_SPEED_SEC)

    def stop(self):
        self._stepper.stop()
