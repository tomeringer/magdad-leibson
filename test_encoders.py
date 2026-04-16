import time
import socket
import pigpio


# --- ENCODER CLASS FROM CHASSIS CODE ---
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


# --- NETWORKING SETUP ---
UDP_IP = "10.135.205.20"  # <--- REPLACE WITH YOUR COMPUTER'S IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- INITIALIZATION ---
pi = pigpio.pi()
enc_left = YellowJacketEncoder(pi, 24, 25)
enc_right = YellowJacketEncoder(pi, 9, 10)

print(f"Streaming data to {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        # Get data
        L_revs = enc_left.output_revolutions()
        R_revs = enc_right.output_revolutions()

        # Format string: "L_val,R_val"
        message = f"{L_revs:.2f},{R_revs:.2f}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        time.sleep(0.05)  # 20Hz update rate
except KeyboardInterrupt:
    pi.stop()