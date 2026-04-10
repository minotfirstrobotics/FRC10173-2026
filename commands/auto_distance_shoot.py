import math
import wpilib
import commands2
from commands2 import cmd
from wpilib import SmartDashboard
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_SwerveDrive import SS_SwerveDrive

class CMD_AutoDistanceShoot(commands2.Command):
    def __init__(self, shooter: SS_Kraken, swerve: SS_SwerveDrive):
        super().__init__()
        self.shooter = shooter
        self.swerve = swerve

        self.addRequirements(self.shooter)

    def get_required_shooter_speed_raw(self) -> float:
        distance = max(self.swerve.range_to_target, 0.1)
        # y=2.25x^{3}-19.78x^{2}+57.88x-18.38
        return (2.25 * distance**3) - (19.78 * distance**2) + (57.88 * distance) - 18.38

    def get_required_shooter_speed(self) -> float:
        return max(min(self.get_required_shooter_speed_raw(), self.shooter.max_rps), -self.shooter.max_rps)

    def update_and_get_required_shooter_speed(self) -> float:
        required_speed = self.get_required_shooter_speed()
        SmartDashboard.putNumber("SS_Telemetry/Shooter/Shooter Auto Distance Speed", required_speed)
        return required_speed

    def apply_required_shooter_speed(self) -> float:
        required_speed = self.update_and_get_required_shooter_speed()
        self.shooter.set_velocity_setpoint(required_speed, publish_dashboard=False)
        self.shooter._run_at_velocity(required_speed)
        return required_speed

    def initialize(self):
        self.apply_required_shooter_speed()

    def execute(self):
        self.apply_required_shooter_speed()

    def isFinished(self):
        return False  # run until interrupted

    def end(self, interrupted: bool):
        self.shooter._stop_motor()