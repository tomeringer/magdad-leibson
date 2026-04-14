import sys
import tty
import termios

from gpiozero.pins.pigpio import PiGPIOFactory
import piano_player as piano

def run_piano_control():
    # Standard helper for raw keyboard input over SSH
    def getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    # Initialize hardware
    piano.init(PiGPIOFactory())

    # Track the state for each finger (0 to 3)
    current_states = [0, 0, 0, 0, 0]

    print("--- Piano Servo SSH Control ---")
    print("Keys 1-5: Cycle Fingers (0->3) | Space: Reset All | Q: Quit")

    # Initial set to flat
    piano.set_states(current_states)

    while True:
        char = getch().lower()

        if char in ['1', '2', '3', '4', '5']:
            index = int(char) - 1
            
            # Cycle through states 0, 1, 2, 3
            current_states[index] = (current_states[index] + 1) % 4

            print(f"\rFinger {char} State: {current_states[index]}      ", end="")
            piano.set_states(current_states)

        elif char == ' ' or char == 'k':
            # Emergency Reset: lift all fingers to state 0
            current_states = [0, 0, 0, 0, 0]
            print("\rResetting all fingers to state 0...      ", end="")
            piano.set_states(current_states)

        elif char == 'q':
            print("\nQuitting...")
            break

if __name__ == "__main__":
    try:
        run_piano_control()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")