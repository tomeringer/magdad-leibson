from gpiozero import PWMOutputDevice

# Constants from your original script
ARM_RPWM_PIN = 14  # Connected to L298N IN3
ARM_LPWM_PIN = 15  # Connected to L298N IN4

# Module State (Starts as None to be filled by init)
ARM_RPWM = None
ARM_LPWM = None


def init(factory):
    """Initializes the L298N pins using the shared factory."""
    global ARM_RPWM, ARM_LPWM

    # We use PWMOutputDevice exactly like the chassis
    ARM_RPWM = PWMOutputDevice(ARM_RPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)
    ARM_LPWM = PWMOutputDevice(ARM_LPWM_PIN, frequency=1000, initial_value=0, pin_factory=factory)


def run(forward: bool) -> None:
    """
    Moves the arm using L298N logic.
    Exactly like drive_forward or drive_reverse.
    """
    if ARM_RPWM and ARM_LPWM:
        if forward:
            ARM_RPWM.value = 1.0
            ARM_LPWM.value = 0.0
        else:
            ARM_RPWM.value = 0.0
            ARM_LPWM.value = 1.0


def stop() -> None:
    """Sets both PWM values to 0 to stop the motor."""
    if ARM_RPWM and ARM_LPWM:
        ARM_RPWM.value = 0.0
        ARM_LPWM.value = 0.0