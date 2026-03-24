import wpilib
import commands2
from subsystems.SS_ShooterKraken import SS_ShooterKraken
from examples.SS_FeederTalon_Power import SS_FeederTalon_Power
from commands2.button import CommandXboxController


class CMD_ComboShoot(commands2.Command):
    def __init__(self, ss_shooter: SS_ShooterKraken, ss_feeder: SS_FeederTalon_Power, joystick: CommandXboxController):
        super().__init__()
        self.ss_shooter = ss_shooter
        self.ss_feeder = ss_feeder
        self.addRequirements(self.ss_shooter, self.ss_feeder) # Ensure no other command these subsystems while this command is running
        self.timer = wpilib.Timer()
        self.velocity_tolerance = 500 # RPM tolerance for considering the shooter "up to speed"
        self._joystick = joystick

    def initialize(self):
        self.timer.restart()
        self.ss_shooter.set_velocity() # Spin up shooter

    def execute(self):
            if abs(self.ss_shooter.current_velocity - self.ss_shooter.setpoint_velocity) < self.velocity_tolerance:
                self.ss_feeder.set_speed(self.ss_feeder.cruising_speed) # Spin up feeder
            else:
                self.ss_feeder.stop_motor() # Stop feeder if shooter is not up to speed

    def isFinished(self):
        return not self._joystick.rightBumper().getAsBoolean() # Run until the right bumper is released


    def end(self, interrupted):
        # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
        if interrupted:
            wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", printTrace=False)
        else:
            wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", printTrace=False)
