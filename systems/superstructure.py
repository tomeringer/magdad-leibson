import time
from typing import Optional
import arm
import chassis
import glove
import gripper
import ultrasonic

class Command:
    def __init__(self, gripper: Optional[int], arm: Optional[(int, int)], chassis: chassis.States):
        self.gripper = gripper
        self.arm = arm
        self.chassis = chassis


command = Command(None, None, chassis.States.STOP)

def set_command(n_command: Command):
    # Ultrasonic safety:
    # If too close (<40cm), BLOCK forward/turn, but allow reverse to back away.
    if ultrasonic.too_close() and n_command.chassis in (chassis.States.FORWARD, chassis.States.TURN_LEFT, chassis.States.TURN_RIGHT):
        n_command.chassis = chassis.States.STOP
        now = time.time()
        if now - glove.last_ultra_block_print > 0.5:
            glove.last_ultra_block_print = now
            print(f"[ULTRA] BLOCK DRIVE: {ultrasonic.last_distance_cm:.1f} cm < {ultrasonic.ULTRA_STOP_CM:.1f} cm")
        return

