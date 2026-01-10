from typing import Optional

from systems.chassis import States


class Command:
    def __init__(
            self,
            _gripper: Optional[int],
            _arm: Optional[int],
            _chassis: States,
    ):
        self.gripper = _gripper
        self.arm = _arm
        self.chassis = _chassis

    def set(self, n_command):
        self.gripper = n_command.gripper
        self.arm = n_command.arm
        self.chassis = n_command.chassis

def empty_command() -> Command:
    return Command(None, None, States.STOP)