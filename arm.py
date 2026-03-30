from gpiozero import Servo

arm = None


def init_arm(factory):
    global arm
    # Note: Pin 15 from your original code
    arm = Servo(15, min_pulse_width=1 / 1000, max_pulse_width=2 / 1000, pin_factory=factory)


def run_arm(forward: bool) -> None:
    if forward:
        arm.value = 1.0
    else:
        arm.value = -1.0


def stop_arm() -> None:
    arm.value = 0
