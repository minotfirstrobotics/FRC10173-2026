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
        # Start shooter spin-up
        self.shooter.spin_up_and_wait().schedule()

    def execute(self):
        # Compute required shooter speed every 20ms
        distance = self.swerve.range_to_target
        required_speed = (5.17 * distance) + 24.9

        # Apply velocity dynamically
        self.shooter._run_at_velocity_injected(required_speed)

    def isFinished(self):
        return False  # runs until interrupted

    def end(self, interrupted: bool):
        self.shooter._stop_motor()

