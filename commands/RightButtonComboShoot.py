import wpilib
import commands2
from subsystems import SS_ShooterNEO
from subsystems import SS_FeederTalon_Power
from commands2.button import CommandXboxController


class SpinUpShooterwithFeeder(commands2.Command):
    def __init__(self, ss_shooter: SS_ShooterNEO, ss_feeder: SS_FeederTalon_Power, joystick: CommandXboxController):
        super().__init__()
        self.ss_shooter = ss_shooter
        self.addRequirements(ss_shooter) # Ensure no other command uses ss_shooter
        self.ss_feeder = ss_feeder
        self.addRequirements(ss_feeder) # Ensure no other command uses ss_feeder
        self.timer = wpilib.Timer()
        self.velocity_tolerance = 100 # RPM tolerance for considering the shooter "up to speed"
        self._joystick = joystick

    def initialize(self):
        self.timer.restart()
        self.ss_shooter.run_setpoint_velocity_command() # Spin up shooter

    def execute(self):
            if self.ss_shooter.current_velocity > 4000:
                 self.ss_feeder.run_forward_command() # Spin up feeder                                  

    def isFinished(self):
        return self._joystick.rightBumper().getAsBoolean() == False # Run until the right bumper is released


    def end(self, interrupted):
        # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
        if interrupted:
            wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", stacktrace=False)
        else:
            wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", stacktrace=False)
