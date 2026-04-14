from gpiozero import Servo

SERVO_PINS = [12, 4, 17, 27 ,22]  # From left to right. TODO: Update according to actual servos
SERVO_MOVE_STEP = -0.144
servo_offsets = [
    (80 - 90) / 90,   # -0.111
    (85 - 90) / 90,   # -0.055
    (120 - 90) / 90,  #  0.333
    (90 - 90) / 90,   #  0.0
    (80 - 90) / 90    # -0.111
]

_current_servos_pos = servo_offsets.copy()
_servos_states = [0, 0, 0, 0, 0]
servos = []

def init(factory):
    global servos
    for pin in SERVO_PINS:
        servos.append(Servo(
            pin,
            min_pulse_width=1 / 1000,
            max_pulse_width=2 / 1000,
            pin_factory=factory
        ))

def move_step_index(position: int, index: int) -> None:
    global _current_servos_pos, _servos_states, servos
    if position == _servos_states[index]:
        return

    new_val = _current_servos_pos[index] + (position - _servos_states[index]) * SERVO_MOVE_STEP

    new_val = max(-1.0, min(1.0, new_val))
    _current_servos_pos[index] = new_val
    servos[index].value = _current_servos_pos[index]
    _servos_states[index] = position

def set_states(positions: list[int]) -> None:
    for i in range(len(positions)):
        move_step_index(positions[i], i)