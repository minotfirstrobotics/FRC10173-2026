from commands2 import SequentialCommandGroup, WaitCommand, cmd
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from examples.SS_ShooterKraken import SS_ShooterKraken
from examples.SS_FeederKraken import SS_FeederKraken
from examples.SS_IntakeKraken import SS_IntakeKraken
from subsystems.SS_CANdleLight import SS_CANdleLight


def SEQ_Shoot(shooter: SS_Kraken, feeder: SS_Kraken):
    return SequentialCommandGroup(
        cmd.runOnce(shooter.spin_up_and_wait_command),
        cmd.runOnce(lambda: shooter.run_velocity_at_setpoint), # Ensure shooter is running at setpoint while feeder runs
        cmd.runOnce(lambda: feeder.run_velocity_at_setpoint).withTimeout(3.0), # Run feeder for 3 seconds after shooter is up to speed
        cmd.runOnce(feeder.stop_motor).withTimeout(3.0), # Run feeder for 3 seconds after shooter is up to speed
        cmd.runOnce(shooter.stop_motor).withTimeout(3.0), # Run feeder for 3 seconds after shooter is up to speed
    )

def SEQ_DeployIntake(swerve: SS_SwerveDrive):
    return SequentialCommandGroup(
        swerve.padlocked_drive_request_command(vx_requested=0.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(2.0),
        swerve.padlocked_drive_request_command(vx_requested=-1.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(1.0),
        swerve.padlocked_drive_request_command(vx_requested=2.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.2),
        swerve.padlocked_drive_request_command(vx_requested=-0.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.5),
    )

