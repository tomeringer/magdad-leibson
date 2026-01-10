SERVO_PIN = 12
SERVO_NEUTRAL_US = 1500
SERVO_SPIN_DELTA_US = 200

try:
    import pigpio
except ImportError:
    pigpio = None


class Gripper:
    def __init__(self):
        self.SERVO_PIN = SERVO_PIN
        self.SERVO_NEUTRAL_US = SERVO_NEUTRAL_US
        self.SERVO_SPIN_DELTA_US = SERVO_SPIN_DELTA_US

        self.pigpio = pigpio
        self._pi = None

    def servo_init(self):
        if self.pigpio is None:
            print("WARNING: pigpio not installed")
            return
        self._pi = self.pigpio.pi()
        if not self._pi.connected:
            print("WARNING: pigpio daemon not running (sudo systemctl enable --now pigpiod)")
            self._pi = None
            return
        self._pi.set_servo_pulsewidth(self.SERVO_PIN, self.SERVO_NEUTRAL_US)

    def servo_stop(self):
        if self._pi:
            self._pi.set_servo_pulsewidth(self.SERVO_PIN, self.SERVO_NEUTRAL_US)

    def servo_spin(self, direction_bit: int):
        """
        direction_bit: 1 or 0 (chooses pulsewidth above/below neutral)
        """
        if self._pi is None or direction_bit is None:
            self.servo_stop()
            return
        pw = (
            self.SERVO_NEUTRAL_US + self.SERVO_SPIN_DELTA_US
            if direction_bit
            else self.SERVO_NEUTRAL_US - self.SERVO_SPIN_DELTA_US
        )
        self._pi.set_servo_pulsewidth(self.SERVO_PIN, pw)

    def servo_cleanup(self):
        if self._pi:
            self.servo_stop()
            self._pi.stop()
            self._pi = None
