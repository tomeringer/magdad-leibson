import time
from typing import Optional

import RPi.GPIO as GPIO


class Ultrasonic:
    def __init__(self):
        # ============================================================
        # ULTRASONIC
        # ============================================================
        self.TRIG = 13
        self.ECHO = 19
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

        self.ULTRA_ENABLED = True
        self.ULTRA_STOP_CM = 40.0

        self.ULTRA_MEASURE_PERIOD_SEC = 0.10  # measure at 10 Hz
        self.ULTRA_PRINT_PERIOD_SEC = 0.50  # print at 2 Hz

        self._last_ultra_t = 0.0
        self._last_ultra_print_t = 0.0
        self.last_distance_cm: Optional[float] = None

    def measure_distance_cm(self) -> Optional[float]:
        """
        Returns distance in cm or None on timeout.
        """
        GPIO.output(self.TRIG, False)
        time.sleep(0.0002)

        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)
        GPIO.output(self.TRIG, False)

        t0 = time.time()
        while GPIO.input(self.ECHO) == 0:
            if time.time() - t0 > 0.03:
                return None

        pulse_start = time.time()
        while GPIO.input(self.ECHO) == 1:
            if time.time() - pulse_start > 0.03:
                return None

        pulse_end = time.time()
        return (pulse_end - pulse_start) * 17150.0

    def tick(self) -> Optional[float]:
        """
        Periodically measures + periodically prints.
        Updates last_distance_cm and returns it.
        """
        now = time.time()
        if not self.ULTRA_ENABLED:
            return self.last_distance_cm

        if now - self._last_ultra_t >= self.ULTRA_MEASURE_PERIOD_SEC:
            self._last_ultra_t = now
            self.last_distance_cm = self.measure_distance_cm()

        if now - self._last_ultra_print_t >= self.ULTRA_PRINT_PERIOD_SEC:
            self._last_ultra_print_t = now
            if self.last_distance_cm is None:
                print("[ULTRA] dist=None (timeout)")
            else:
                print(f"[ULTRA] dist={self.last_distance_cm:.1f} cm")

        return self.last_distance_cm

    def too_close(self) -> bool:
        return (
                self.ULTRA_ENABLED
                and (self.last_distance_cm is not None)
                and (self.last_distance_cm < self.ULTRA_STOP_CM)
        )
