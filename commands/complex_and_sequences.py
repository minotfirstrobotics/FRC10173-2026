import wpilib
import commands2
from commands2 import WaitCommand, cmd
from commands2.button import CommandXboxController
from wpilib import SmartDashboard
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_CANdleLight import SS_CANdleLight


def SEQ_shoot(shooter: SS_Kraken, feeder: SS_Kraken):
    return commands2.SequentialCommandGroup(
        shooter.spin_up_and_wait(),
        commands2.ParallelDeadlineGroup(
            WaitCommand(3.0),
            shooter.hold_velocity(shooter.get_velocity_setpoint),
            feeder.hold_velocity(feeder.get_dashboard_velocity_setpoint),
        ),
        cmd.runOnce(feeder._stop_motor),
        cmd.runOnce(shooter._stop_motor),
    )

def SEQ_extend_intake(extender: SS_Kraken):
    return commands2.SequentialCommandGroup(
        cmd.runOnce(lambda: extender._rotate_to_position(1.5), extender),
        WaitCommand(2.0),
        cmd.runOnce(extender._stop_motor)
    )

def SEQ_auto_shake_intake(swerve: SS_SwerveDrive):
    return commands2.SequentialCommandGroup(
        swerve.padlocked_drive_request_command(vx_requested=0.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(2.0),
        swerve.padlocked_drive_request_command(vx_requested=-1.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(1.0),
        swerve.padlocked_drive_request_command(vx_requested=2.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.2),
        swerve.padlocked_drive_request_command(vx_requested=-0.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.5),
    )

def CMD_deploy_intake(extender: SS_Kraken, shooter: SS_Kraken):
    return commands2.SequentialCommandGroup(
        commands2.ParallelDeadlineGroup(
            WaitCommand(5.0),
            cmd.run(lambda: extender._rotate_to_position(3), extender),
            cmd.run(
                lambda: shooter._run_power_percent(-abs(shooter.percent_power_setpoint)),
                shooter,
            ),
        ),
        cmd.runOnce(extender._stop_motor, extender),
        cmd.runOnce(shooter._stop_motor, shooter),
    )

class CMD_ComboShoot(commands2.Command):
    def __init__(self, ss_shooter: SS_Kraken, ss_feeder: SS_Kraken, ss_swerve: SS_SwerveDrive, joystick: CommandXboxController | None = None,):
        super().__init__()
        self.ss_shooter = ss_shooter
        self.ss_feeder = ss_feeder
        self.ss_swerve = ss_swerve
        self.addRequirements(self.ss_shooter, self.ss_feeder) # Allow auto path following to keep control of swerve while shooting
        self._joystick = joystick

    def initialize(self):
        range_in_meters = self.ss_swerve.range_to_target
        shooter_setpoint = 5.17 * range_in_meters + 24.9
        self.ss_shooter.set_velocity_setpoint(shooter_setpoint)
        SmartDashboard.putNumber("Shooter Setpoint", shooter_setpoint)
        self.ss_shooter._run_at_velocity(shooter_setpoint) # Spin up shooter

    def execute(self):
            if self.ss_shooter.is_at_velocity():
                feeder_setpoint = self.ss_feeder.set_velocity_setpoint(
                    self.ss_feeder.get_dashboard_velocity_setpoint(),
                    publish_dashboard=False,
                )
                self.ss_feeder._run_at_velocity(feeder_setpoint) # Spin feeder using dashboard velocity directly
            else:
                self.ss_feeder._stop_motor() # Stop feeder if shooter is no longer at speed

    def isFinished(self):
        if self._joystick is None:
            return False
        return not self._joystick.x().getAsBoolean()

    def end(self, interrupted):
        self.ss_shooter._stop_motor()
        self.ss_feeder._stop_motor()
