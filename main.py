import RPi.GPIO as GPIO

from systems import superstructure
from systems.superstructure import Superstructure

MODE = superstructure.Mode.KEYBOARD


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    _super = None
    try:
        _super = Superstructure(MODE)
        _super.start()
        while True:
            _super.tick()
    finally:
        if _super is not None:
            _super.end()
        GPIO.cleanup()



if __name__ == "__main__":
    main()
    