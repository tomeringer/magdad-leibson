from gpiozero import OutputDevice

# Using the pins defined in your original constants
ARM_RPWM_PIN = 14  # IN3
ARM_LPWM_PIN = 15  # IN4

# Module State
in3 = None
in4 = None

def init(factory):
    """Initializes the L298N pins using the shared factory."""
    global in3, in4
    # We use OutputDevice for simple high/low direction control
    in3 = OutputDevice(ARM_RPWM_PIN, pin_factory=factory)
    in4 = OutputDevice(ARM_LPWM_PIN, pin_factory=factory)

def run(forward: bool) -> None:
    """Sets the L298N logic to move the arm."""
    if in3 and in4:
        if forward:
            in3.on()
            in4.off()
        else:
            in3.off()
            in4.on()

def stop() -> None:
    """Cuts power to the motor."""
    if in3 and in4:
        in3.off()
        in4.off()