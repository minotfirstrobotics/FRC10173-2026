from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from examples.SS_ShooterNEO import SS_ShooterNEO, SpinUpAndWait_CommDef
from examples.SS_TurretTalon_Trapezoidal import SS_TurretTalon
from examples.SS_FeederTalon_Power import SS_FeederTalon_Power
from examples.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose


def SEQ_Shoot(shooter: SS_ShooterNEO, feeder: SS_FeederTalon_Power):
    return SequentialCommandGroup(
        shooter.spin_up_and_wait_command(),
        shooter.run_setpoint_velocity_command(), # Ensure shooter is running at setpoint while feeder runs
        feeder.run_forward_command().withTimeout(3.0) # Run feeder for 3 seconds after shooter is up to speed
    )

def SEQ_DeployIntake(swerve: SS_SwerveDrive, shooter: SS_ShooterNEO, feeder: SS_FeederTalon_Power, intake: SS_IntakeSIMM):
    return SequentialCommandGroup(
        swerve.auto_drive_request_command(vx_requested=0.5, vy_requested=0, x_vector=0, y_vector=0).withTimeout(4.0),
        swerve.auto_drive_request_command(vx_requested=-1.5, vy_requested=0, x_vector=0, y_vector=0).withTimeout(1.0),
        swerve.auto_drive_request_command(vx_requested=2, vy_requested=0, x_vector=0, y_vector=0).withTimeout(0.2),
        swerve.heading_is_driver_controlled_command()
    )
