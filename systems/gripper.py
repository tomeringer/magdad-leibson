SERVO_PIN = 12
SERVO_NEUTRAL_US = 1500
SERVO_SPIN_DELTA_US = 200

try:
    import pigpio
except ImportError:
    pigpio = None

_pi = None


def servo_init():
    global _pi
    if pigpio is None:
        print("WARNING: pigpio not installed")
        return
    _pi = pigpio.pi()
    if not _pi.connected:
        print("WARNING: pigpio daemon not running (sudo systemctl enable --now pigpiod)")
        _pi = None
        return
    _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)


def servo_stop():
    if _pi:
        _pi.set_servo_pulsewidth(SERVO_PIN, SERVO_NEUTRAL_US)


def servo_spin(direction_bit: int):
    """
    direction_bit: 1 or 0 (chooses pulsewidth above/below neutral)
    """
    if _pi is None:
        return
    pw = SERVO_NEUTRAL_US + SERVO_SPIN_DELTA_US if direction_bit else SERVO_NEUTRAL_US - SERVO_SPIN_DELTA_US
    _pi.set_servo_pulsewidth(SERVO_PIN, pw)


def servo_cleanup():
    global _pi
    if _pi:
        servo_stop()
        _pi.stop()
        _pi = None