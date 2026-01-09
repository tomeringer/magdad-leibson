from systems import superstructure
from systems.superstructure import Superstructure
import RPi.GPIO as GPIO

MODE = superstructure.Mode.KEYBOARD

def main():
    super = Superstructure(MODE)

    try:
        super.start()
        while True:
            super.tick()
    finally:
        super.end()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
