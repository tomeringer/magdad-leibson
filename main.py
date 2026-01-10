import RPi.GPIO as GPIO

from systems import superstructure
from systems.superstructure import Superstructure

MODE = superstructure.Mode.KEYBOARD


def main():
    _super = Superstructure(MODE)

    try:
        _super.start()
        while True:
            _super.tick()
    finally:
        _super.end()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
