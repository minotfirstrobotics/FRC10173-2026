import math
import wpilib
import commands2
from commands2 import cmd
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_SwerveDrive import SS_SwerveDrive

class CMD_AutoDistanceShoot(commands2.Command):
    def __init__(self, shooter: SS_Kraken, swerve: SS_SwerveDrive):
        super().__init__()
        self.shooter = shooter
        self.swerve = swerve

        self.addRequirements(self.shooter, self.swerve)

    def initialize(self):
        # Start shooter
        self.shooter.spin_up_and_wait().schedule()

    def execute(self):
        # 26.2 + 12.1 ln x
        # required shooter speed
        distance = self.swerve.range_to_target
        required_speed = 26.2 + (12.1 * math.log(distance))

        # Apply velocity 
        self.shooter._run_at_velocity_injected(required_speed)

    def isFinished(self):
        return False  # run until interrupted

    def end(self, interrupted: bool):
        self.shooter._stop_motor()

