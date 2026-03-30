from gpiozero import Servo

motor = None

def init(factory):
    global motor
    motor = Servo(15, min_pulse_width=1 / 1000, max_pulse_width=2 / 1000, pin_factory=factory)

def run(forward: bool) -> None:
    if motor:
        motor.value = 1.0 if forward else -1.0

def stop() -> None:
    if motor:
        motor.value = 0