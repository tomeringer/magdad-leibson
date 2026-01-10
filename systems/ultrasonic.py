import time
import RPi.GPIO as GPIO


class Ultrasonic:
    def __init__(self):
        # Ultrasonic 1
        self.TRIG_1 = 5
        self.ECHO_1 = 6

        # Ultrasonic 2
        self.TRIG_2 = 13
        self.ECHO_2 = 19

        # ================= CONSTANT =================
        self.BASELINE_D = 37.5  # cm
        self.MINIMAL_D = 50     # cm
        self.TIME_SLEEP = 0.06

        # GPIO setup
        GPIO.setup(self.TRIG_1, GPIO.OUT)
        GPIO.setup(self.ECHO_1, GPIO.IN)
        GPIO.setup(self.TRIG_2, GPIO.OUT)
        GPIO.setup(self.ECHO_2, GPIO.IN)

        GPIO.output(self.TRIG_1, False)
        GPIO.output(self.TRIG_2, False)

        self.drive = True

    # ================= MEASUREMENT =================
    def measure_distance(self, trig, echo, timeout=0.02):
        GPIO.output(trig, True)
        time.sleep(0.00001)
        GPIO.output(trig, False)

        start_time = time.time()

        while GPIO.input(echo) == 0:
            if time.time() - start_time > timeout:
                return None
        pulse_start = time.time()

        while GPIO.input(echo) == 1:
            if time.time() - pulse_start > timeout:
                return None
        pulse_end = time.time()

        duration = pulse_end - pulse_start
        return (duration * 34300) / 2  # cm

    # ================= DRIVING CONDITION =================
    def can_drive(self, l1, l2):
        if l1 < self.MINIMAL_D or l2 < self.MINIMAL_D:
            return False
        return True

    # ================= MAIN LOOP =================
    def tick(self):
        try:
            l1 = self.measure_distance(self.TRIG_1, self.ECHO_1)
            time.sleep(self.TIME_SLEEP)
            l2 = self.measure_distance(self.TRIG_2, self.ECHO_2)

            self.drive = True
            if l1 is not None and l2 is not None:
                self.drive = self.can_drive(l1, l2)

            # -------- Pretty printing --------
            print("===================================")
            print(f"US1 distance: {l1:6.1f} cm" if l1 else "US1 distance: timeout")
            print(f"US2 distance: {l2:6.1f} cm" if l2 else "US2 distance: timeout")
            print("DRIVE:", "YES" if self.drive else "NO")
            print("===================================\n")

        except KeyboardInterrupt:
            print("\nStopping...")
            GPIO.cleanup()
