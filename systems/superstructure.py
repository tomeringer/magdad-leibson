import time
from enum import Enum
from typing import Optional

import arm
import chassis
import glove
import gripper
import ultrasonic
from systems import keyboard


class Command:
    def __init__(
            self,
            _gripper: Optional[int],
            _arm: Optional[(int, int)],
            _chassis: chassis.States,
    ):
        self.gripper = _gripper
        self.arm = _arm
        self.chassis = _chassis

    def set(self, n_command):
        self.gripper = n_command.gripper
        self.arm = n_command.arm
        self.chassis = n_command.chassis


def empty_command() -> Command:
    return Command(None, None, chassis.States.STOP)


class Mode(Enum):
    GLOVE = 1
    KEYBOARD = 2


class Superstructure:
    def __init__(self, mode: Mode):
        self.mode = mode

        self.gripper = gripper.Gripper()
        self.gripper.servo_init()
        self.arm = arm.Arm()
        self.chassis = chassis.Chassis()

        self.ultrasonic = ultrasonic.Ultrasonic()
        self.glove = glove.GloveController()
        self.keyboard = keyboard.KeyboardController()

        self.command = empty_command()

        self.CONTROL_PERIOD_SEC = 0.01

    def set_command(self, n_command: Command):
        # Ultrasonic safety:
        # If too close (<40cm), BLOCK forward/turn, but allow reverse to back away.
        if self.ultrasonic.too_close() and n_command.chassis in (
                chassis.States.FORWARD,
                chassis.States.TURN_LEFT,
                chassis.States.TURN_RIGHT,
        ):
            n_command.chassis = chassis.States.STOP
            now = time.time()
            if now - self.glove.last_ultra_block_print > 0.5:
                self.glove.last_ultra_block_print = now
                print(
                    f"[ULTRA] BLOCK DRIVE: "
                    f"{self.ultrasonic.last_distance_cm:.1f} cm "
                    f"< {self.ultrasonic.ULTRA_STOP_CM:.1f} cm"
                )
            return

        self.command = n_command

    def start(self):
        if self.mode == Mode.GLOVE:
            self.glove.start()
        else:
            print("[KEYBOARD] Control mode")
            print("Commands: w/s/a/d (drive), x (stop), i/k (servo), o (servo stop), u/j (stepper), q (quit)")

    def tick(self):
        self.ultrasonic.tick()
        if self.mode == Mode.GLOVE:
            self.glove.tick()
            self.set_command(self.glove.command)
        else:
            self.keyboard.tick()
            self.set_command(self.keyboard.command)

        self.gripper.servo_spin(self.command.gripper)
        self.arm.stepper_move(self.command.arm[0], self.command.arm[1])
        self.chassis.run_desired(self.command.chassis)

        time.sleep(self.CONTROL_PERIOD_SEC)

    def stop(self):
        self.gripper.servo_stop()
        self.arm.stop()
        self.chassis.motor_stop()

    def end(self):
        self.stop()
        self.gripper.servo_cleanup()
        self.glove.stop()

