import time
import pigpio

class YellowJacketEncoder:
    """
    Quadrature encoder reader for goBILDA Yellow Jacket motor
    Encoder is on motor shaft (before gearbox)
    """

    # Gray-code transition table
    _TRANS = {
        0b0001: +1, 0b0010: -1,
        0b0100: -1, 0b0111: +1,
        0b1000: +1, 0b1011: -1,
        0b1101: -1, 0b1110: +1,
    }

    def __init__(self, pi, gpio_a, gpio_b, gear_ratio=13.7):
        self.pi = pi
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b

        # Yellow Jacket constants
        self.ppr_motor = 28
        self.counts_per_rev_motor = self.ppr_motor * 4
        self.gear_ratio = gear_ratio
        self.counts_per_rev_output = self.counts_per_rev_motor * gear_ratio

        self.counts = 0
        self._last_state = 0
        self._t_last = time.time()
        self._c_last = 0
        self._motor_rpm = 0.0

        for g in (gpio_a, gpio_b):
            self.pi.set_mode(g, pigpio.INPUT)
            self.pi.set_pull_up_down(g, pigpio.PUD_UP)

        a = self.pi.read(gpio_a)
        b = self.pi.read(gpio_b)
        self._last_state = (a << 1) | b

        self._cba = self.pi.callback(gpio_a, pigpio.EITHER_EDGE, self._cb)
        self._cbb = self.pi.callback(gpio_b, pigpio.EITHER_EDGE, self._cb)

    def _cb(self, gpio, level, tick):
        a = self.pi.read(self.gpio_a)
        b = self.pi.read(self.gpio_b)
        new_state = (a << 1) | b
        key = (self._last_state << 2) | new_state
        self.counts += self._TRANS.get(key, 0)
        self._last_state = new_state

    def update(self, window_s=0.2):
        t = time.time()
        dt = t - self._t_last
        if dt < window_s:
            return

        dc = self.counts - self._c_last
        motor_rps = (dc / self.counts_per_rev_motor) / dt
        self._motor_rpm = 60 * motor_rps

        self._t_last = t
        self._c_last = self.counts

    # ===== Public API =====

    def motor_rpm(self):
        return self._motor_rpm

    def output_rpm(self):
        return self._motor_rpm / self.gear_ratio

    def output_revolutions(self):
        return self.counts / self.counts_per_rev_output

    def output_degrees(self):
        return self.output_revolutions() * 360.0

    def stop(self):
        self._cba.cancel()
        self._cbb.cancel()


def main():
    ENC_A = 24  # free
    ENC_B = 25  # free

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running")

    enc = YellowJacketEncoder(pi, ENC_A, ENC_B, gear_ratio=13.7)

    try:
        while True:
            enc.update()
            print(
                f"counts={enc.counts:7d} | "
                f"motor RPM={enc.motor_rpm():7.2f} | "
                f"output RPM={enc.output_rpm():6.2f} | "
                f"output angle={enc.output_degrees():8.2f}°"
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        enc.stop()
        pi.stop()


if __name__ == "__main__":
    main()
