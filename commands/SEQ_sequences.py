from commands2 import SequentialCommandGroup, WaitCommand, cmd
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from examples.SS_ShooterNEO import SS_ShooterNEO, SpinUpAndWait_CommDef
from examples.SS_FeederTalon_Power import SS_FeederTalon_Power
from examples.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight


def SEQ_Shoot(shooter: SS_ShooterNEO, feeder: SS_FeederTalon_Power):
    return SequentialCommandGroup(
        cmd.runOnce(shooter.spin_up_and_wait_command),
        cmd.runOnce(lambda: shooter.set_velocity_to_setpoint(shooter.setpoint_velocity)), # Ensure shooter is running at setpoint while feeder runs
        cmd.runOnce(lambda: feeder.set_speed(feeder.cruising_speed)).withTimeout(3.0), # Run feeder for 3 seconds after shooter is up to speed
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

