#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class StepperMotor:
    def __init__(self, name, pins):
        self.name = name
        self.pins = pins

        # Full-step sequence (4 states)
        self.seq = [
            [1, 0, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 0, 1],
            [1, 0, 0, 1],
        ]

        # Setup pins
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

    def move(self, steps, direction=1, speed=0.005):
        """
        Rotate the motor.
        steps: number of step-cycles to run
        direction: 1 (one way), -1 (the other way)
        speed: delay between micro-states in seconds (smaller=faster)
        """
        if direction not in (1, -1):
            raise ValueError("direction must be 1 or -1")

        print(f"{self.name} moving {steps} steps, direction={direction}, speed={speed}s")

        try:
            for _ in range(steps):
                for step in range(len(self.seq)):
                    step_index = step if direction == 1 else (len(self.seq) - 1 - step)

                    # Output the coil pattern
                    pattern = self.seq[step_index]
                    for i, pin in enumerate(self.pins):
                        GPIO.output(pin, pattern[i])

                    time.sleep(speed)
        finally:
            # Turn off coils even if interrupted mid-move
            self.stop()

    def stop(self):
        for pin in self.pins:
            GPIO.output(pin, 0)

# --- CONFIG: your one motor pins ---
MOTOR_PINS = [17, 18, 27, 22]   # change if needed

motor = StepperMotor("Motor", MOTOR_PINS)

try:
    # Example moves
    motor.move(200, direction=1, speed=0.005)
    time.sleep(0.5)
    motor.move(200, direction=-1, speed=0.005)

except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C)")

finally:
    motor.stop()
    GPIO.cleanup()
    print("GPIO cleanup finished")
