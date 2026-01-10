from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory


class Gripper:
    def __init__(
        self,
        servo_pin: int = 18,
        move_step: float = 0.39,   # ~70 degrees for 360 servo
        zero_pos: float = -0.95,  # calibrated physical zero
    ):
        # --- Hardware & calibration ---
        self.SERVO_PIN = servo_pin
        self.MOVE_STEP = move_step
        self.MY_ZERO = zero_pos

        self.factory = PiGPIOFactory()
        self.servo = Servo(
            self.SERVO_PIN,
            initial_value=None,
            min_pulse_width=0.5 / 1000,
            max_pulse_width=2.5 / 1000,
            pin_factory=self.factory,
        )

        self.current_pos = self.MY_ZERO

    def open_servo(self):
        """מבצע תנועת פתיחה של ~70 מעלות"""
        new_val = self.current_pos + self.MOVE_STEP
        if new_val > 1.0:
            new_val = 1.0

        self.current_pos = new_val
        self.servo.value = self.current_pos

    def close_servo(self):
        """מבצע תנועת סגירה של ~70 מעלות"""
        new_val = self.current_pos - self.MOVE_STEP
        if new_val < -1.0:
            new_val = -1.0

        self.current_pos = new_val
        self.servo.value = self.current_pos

    def stop_servo(self):
        """משחרר את המנוע (מומלץ לקרוא לזה בסיום התוכנית הכללית)"""
        self.servo.detach()
