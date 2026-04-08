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

    def get_required_shooter_speed(self) -> float:
        distance = max(self.swerve.range_to_target, 0.1)
        # 26.2 + 12.1 ln x
        return 26.2 + (12.1 * math.log(distance))

    def initialize(self):
        required_speed = self.get_required_shooter_speed()
        self.shooter.set_velocity_setpoint(required_speed)
        self.shooter._run_at_velocity(required_speed)

    def execute(self):
        required_speed = self.get_required_shooter_speed()
        self.shooter.set_velocity_setpoint(required_speed)
        self.shooter._run_at_velocity(required_speed)

    def isFinished(self):
        return False  # run until interrupted

    def end(self, interrupted: bool):
        self.shooter._stop_motor()