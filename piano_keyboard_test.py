import sys
import tty
import termios
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

    # Initialize hardware (using None for default factory)
    piano.init(None)

    # We keep track of the toggle state for each finger
    current_states = [0, 0, 0, 0, 0]

    print("--- Piano Servo SSH Control ---")
    print("Keys 1-5: Toggle Fingers | Space: Reset All | Q: Quit")

    while True:
        char = getch().lower()

        if char in ['1', '2', '3', '4', '5']:
            index = int(char) - 1
            # Toggle between 0 and 1
            current_states[index] = 1 if current_states[index] == 0 else 0

            print(f"\rFinger {char} State: {current_states[index]}", end="")
            piano.set_states(current_states)

        elif char == ' ' or char == 'k':
            # Emergency Reset: lift all fingers
            current_states = [0, 0, 0, 0, 0]
            print("\rResetting all fingers...       ", end="")
            piano.set_states(current_states)

        elif char == 'q':
            print("\nQuitting...")
            break


if __name__ == "__main__":
    try:
        run_piano_control()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")