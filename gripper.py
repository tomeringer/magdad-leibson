from gpiozero import Servo

SERVO_PIN = 12
SERVO_MOVE_STEP = 0.39

_current_servo_pos = 0.0
_servo_is_open = True
servo_obj = None  # Renamed to avoid naming conflict with the class Servo

def init(factory):
    global servo_obj
    servo_obj = Servo(
        SERVO_PIN,
        min_pulse_width=0.5 / 1000,
        max_pulse_width=2.5 / 1000,
        pin_factory=factory
    )

def move_step(direction: int) -> None:
    global _current_servo_pos, _servo_is_open
    want_close = (direction == 1)
    if want_close and not _servo_is_open: return
    if not want_close and _servo_is_open: return

    if want_close:
        new_val = _current_servo_pos + SERVO_MOVE_STEP
    else:
        new_val = _current_servo_pos - SERVO_MOVE_STEP

    new_val = max(-1.0, min(1.0, new_val))
    _current_servo_pos = new_val
    servo_obj.value = _current_servo_pos
    _servo_is_open = not want_close

def close_pins():
    global servo_obj
    if servo_obj is not None:
        servo_obj.detach() 
        import time
        time.sleep(0.1)
        servo_obj.close()  