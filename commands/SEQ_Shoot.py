from commands2 import SequentialCommandGroup, WaitCommand
from subsystems.SS_FeederTalon_Power import SS_FeederTalon_Power
from subsystems.SS_ShooterKraken import SS_ShooterKraken, SpinUpAndWait_CommDef

class SEQ_Shoot(SequentialCommandGroup):
    """

    """
    def __init__(self, shooter: SS_ShooterKraken, feeder: SS_FeederTalon_Power):
        super().__init__(
            SpinUpAndWait_CommDef(shooter),
            feeder.run_forward_command().withTimeout(3.0) # Run feeder for 3 seconds after shooter is up to speed
        )
