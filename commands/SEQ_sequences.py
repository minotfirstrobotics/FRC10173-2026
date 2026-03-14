from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.SS_FeederTalon_Power import SS_FeederTalon_Power
from subsystems.SS_ShooterNEO import SS_ShooterNEO, SpinUpAndWait_CommDef
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterNEO import SS_ShooterNEO
from subsystems.SS_TurretTalon_Trapezoidal import SS_TurretTalon
from subsystems.SS_FeederTalon_Power import SS_FeederTalon_Power
from subsystems.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose

class SEQ_Shoot(SequentialCommandGroup):
    """

    """
    def __init__(self, shooter: SS_ShooterNEO, feeder: SS_FeederTalon_Power):
        super().__init__(
            SpinUpAndWait_CommDef(shooter),
            feeder.run_forward_command().withTimeout(3.0) # Run feeder for 3 seconds after shooter is up to speed
        )


class SEQ_DeployIntake(SequentialCommandGroup):
    """

    """
    def __init__(self, swerve: SS_SwerveDrive, shooter: SS_ShooterNEO, feeder: SS_FeederTalon_Power, intake: SS_IntakeSIMM):
        super().__init__(
            SequentialCommandGroup(
                swerve.heading_is_auto_controlled_command(vx_requested=1.5, vy_requested=0, vrotation_requested=0).withTimeout(2), # Briefly drive forward to deploy intake
                swerve.heading_is_auto_controlled_command(vx_requested=-5, vy_requested=0, vrotation_requested=0).withTimeout(.5), # Briefly drive forward to deploy intake
                swerve.heading_is_auto_controlled_command(vx_requested=2, vy_requested=0, vrotation_requested=0).withTimeout(.1), # Briefly drive forward to deploy intake
                swerve.heading_is_auto_padlocked_command(vx_requested=0, vy_requested=0, x_vector=0, y_vector=.2), # Briefly drive forward to deploy intake
            )
        )
