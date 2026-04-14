from gpiozero import Servo

# Reordered to strictly match Finger 1, Finger 2, Finger 3, Finger 4, Finger 5
SERVO_PINS = [12, 4, 22, 17, 27]  # Updated to match the new pin order

SERVO_OFFSETS = [95, 85, 85, 87.5, 120]
SERVO_STATES = [0, -13, -27, -40]

servos = []

def init(factory):
    global servos
    for pin in SERVO_PINS:
        servos.append(Servo(
            pin,
            # Matched to ESP32's 410 duty (0.5ms) and 1966 duty (2.4ms)
            min_pulse_width=0.5 / 1000,
            max_pulse_width=2.4 / 1000,
            pin_factory=factory
        ))

def move_step_index(state: int, index: int) -> None:
    global servos
    
    # 1. Calculate angle exactly like the ESP32
    base_angle = SERVO_STATES[state]
    calibrated_angle = base_angle + SERVO_OFFSETS[index]

    # 2. Constrain to 0-180 just like ESP32
    calibrated_angle = max(0, min(180, calibrated_angle))

    # 3. Convert 0-180 degrees to gpiozero's required -1.0 to 1.0 range
    # 0 deg = -1.0 | 90 deg = 0.0 | 180 deg = 1.0
    val = (calibrated_angle - 90) / 90.0

    servos[index].value = val

def set_states(positions: list[int]) -> None:
    for i in range(len(positions)):
        move_step_index(positions[i], i)

def close_pins():
    global servos
    for servo in servos:
        servo.close()