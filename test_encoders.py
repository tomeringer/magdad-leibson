import time
import socket
import pigpio
from gpiozero import PWMOutputDevice
from gpiozero.pins.rpigpio import RPiGPIOFactory

# --- CONFIGURATION ---
COMPUTER_IP = "10.135.205.20"  # <--- Change to your PC's IP
UDP_PORT = 5005
factory = RPiGPIOFactory()
pi_enc = pigpio.pi()

# Motor Pins
RIGHT_RPWM_PIN, RIGHT_LPWM_PIN = 2, 3
LEFT_RPWM_PIN, LEFT_LPWM_PIN = 20, 21
ENC_LEFT_A, ENC_LEFT_B = 24, 25
ENC_RIGHT_A, ENC_RIGHT_B = 9, 10

# Constants
SPEED_DIFF_FACTOR = 1.07


# --- ENCODER CLASS ---
class YellowJacketEncoder:
    _TRANS = {0b0001: +1, 0b0010: -1, 0b0100: -1, 0b0111: +1,
              0b1000: +1, 0b1011: -1, 0b1101: -1, 0b1110: +1}

    def __init__(self, pi, gpio_a, gpio_b):
        self.pi, self.gpio_a, self.gpio_b = pi, gpio_a, gpio_b
        self.ppr_output = 384.5 * (100 / 106)
        self.counts = 0
        self._last_state = (pi.read(gpio_a) << 1) | pi.read(gpio_b)
        self._cba = pi.callback(gpio_a, 3, self._cb)
        self._cbb = pi.callback(gpio_b, 3, self._cb)

    def _cb(self, gpio, level, tick):
        new_state = (self.pi.read(self.gpio_a) << 1) | self.pi.read(self.gpio_b)
        self.counts += self._TRANS.get((self._last_state << 2) | new_state, 0)
        self._last_state = new_state

    def output_revolutions(self): return self.counts / self.ppr_output


# --- INITIALIZATION ---
# Motors
right_rpwm = PWMOutputDevice(RIGHT_RPWM_PIN, frequency=1000, pin_factory=factory)
left_rpwm = PWMOutputDevice(LEFT_RPWM_PIN, frequency=1000, pin_factory=factory)
# (Left/Right LPWM are only needed for reverse; omitted here for forward-only test)

# Encoders
enc_left = YellowJacketEncoder(pi_enc, ENC_LEFT_A, ENC_LEFT_B)
enc_right = YellowJacketEncoder(pi_enc, ENC_RIGHT_A, ENC_RIGHT_B)

# Network
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def drive_forward(speed=0.3):
    right_rpwm.value = speed * SPEED_DIFF_FACTOR
    left_rpwm.value = speed


def stop_motors():
    right_rpwm.value = 0
    left_rpwm.value = 0


# --- MAIN LOOP ---
try:
    print("Moving forward and streaming data...")
    drive_forward(0.25)  # Set speed to 25%

    while True:
        # 1. Read Encoders
        l_revs = enc_left.output_revolutions()
        r_revs = enc_right.output_revolutions()

        # 2. Send via UDP
        msg = f"{l_revs:.2f},{r_revs:.2f}"
        sock.sendto(msg.encode(), (COMPUTER_IP, UDP_PORT))

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    stop_motors()
    pi_enc.stop()