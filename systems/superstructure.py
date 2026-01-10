import time
from enum import Enum

from systems import arm
from systems import chassis
from systems import glove
from systems import gripper
from systems import keyboard
from systems import ultrasonic
from systems.command import Command
from systems.command import empty_command


class Mode(Enum):
    GLOVE = 1
    KEYBOARD = 2


class Superstructure:
    def __init__(self, mode: Mode):
        self.mode = mode

        self.gripper = gripper.Gripper()
        self.arm = arm.Arm()
        self.chassis = chassis.Chassis()

        self.ultrasonic = ultrasonic.Ultrasonic()
        self.glove = glove.GloveController()
        self.keyboard = keyboard.KeyboardController()

        self.command = empty_command()
        self.last_command = empty_command()

        self.CONTROL_PERIOD_SEC = 0.01

    def set_command(self, n_command: Command):
        # Ultrasonic safety:
        # If too close (<40cm), BLOCK forward/turn, but allow reverse to back away.
        if not self.ultrasonic.drive and n_command.chassis in (
                chassis.States.FORWARD,
                chassis.States.TURN_LEFT,
                chassis.States.TURN_RIGHT,
        ):
            n_command.chassis = chassis.States.STOP

        self.command.set(n_command=n_command)

    def start(self):
        if self.mode == Mode.GLOVE:
            self.glove.start()
        else:
            print("[KEYBOARD] Control mode")
            print("Commands: w/s/a/d (drive), x (stop), i/k (servo), o (servo stop), u/j (stepper), q (quit)")

    def tick(self):
        self.last_command.set(self.command)
        self.ultrasonic.tick()
        if self.mode == Mode.GLOVE:
            self.glove.tick()
            self.set_command(self.glove.command)

            if self.command.gripper:
                self.gripper.open_servo()
            else:
                self.gripper.close_servo()
            self.arm.stepper_move(self.arm.STEPPER_STEP_CHUNK, self.command.arm)
            self.chassis.run_desired(self.command.chassis)

        else:
            self.keyboard.tick()

            if not self.command.equal(self.last_command):
                self.set_command(self.keyboard.command)

                if self.command.gripper:
                    self.gripper.open_servo()
                else:
                    self.gripper.close_servo()
                self.arm.stepper_move(self.arm.STEPPER_STEP_CHUNK, self.command.arm)
                self.chassis.run_desired(self.command.chassis)


        time.sleep(self.CONTROL_PERIOD_SEC)

    def stop(self):
        self.gripper.stop_servo()
        self.arm.stepper_release()
        self.chassis.motor_stop()

    def end(self):
        self.stop()
        self.glove.stop()
