from systems.chassis import States
from systems.superstructure import empty_command


class KeyboardController:
    def __init__(self):
        self.command = empty_command()

    def tick(self):
        try:
            cmd = input("cmd> ").strip()
            if not cmd:
                return

            c = cmd[0]

            if c == "x":
                print("[KEYBOARD] stop all")

            # DC motors
            elif c == "w":
                self.command.chassis = States.FORWARD
                print("[KEYBOARD] forward")
            elif c == "s":
                self.command.chassis = States.REVERSE
                print("[KEYBOARD] reverse")
            elif c == "a":
                self.command.chassis = States.TURN_LEFT
                print("[KEYBOARD] turn left (in place)")
            elif c == "d":
                self.command.chassis = States.TURN_RIGHT
                print("[KEYBOARD] turn right (in place)")

            # Servo
            elif c == "i":
                self.command.gripper = 1
                print("[KEYBOARD] servo spin dir=1")
            elif c == "k":
                self.command.gripper = 0
                print("[KEYBOARD] servo spin dir=0")
            elif c == "o":
                self.command.gripper = None
                print("[KEYBOARD] servo stop")

            # Stepper
            elif c == "u":
                self.command.arm = 1
                print("[KEYBOARD] stepper forward")
            elif c == "j":
                self.command.arm = 0
                print("[KEYBOARD] stepper reverse")
            else:
                print("[KEYBOARD] unknown command")

        except KeyboardInterrupt:
            pass
