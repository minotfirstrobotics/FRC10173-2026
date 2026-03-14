import commands2
from commands2.button import CommandXboxController
from commands import CMD_Shoot

from subsystems import (
    SS_FeederTalon_Power,
    SS_TurretTalon_Trapezoidal,
    SS_ShooterNEO,
    SS_CameraPose,
    SS_SwerveDrive,
    SS_IntakeSIMM,
    SS_CANdleLight
)


# this subsystem is an all in one for controller inputs and binding commands
class SS_ControllerSetInputs(commands2.Subsystem):

    def __init__(
        self,
        driver: CommandXboxController,
        operator: CommandXboxController,
        feeder: SS_FeederTalon_Power,
        turret: SS_TurretTalon_Trapezoidal,
        shooter: SS_ShooterNEO,
        camera: SS_CameraPose,
        swerve: SS_SwerveDrive,
        intake: SS_IntakeSIMM,
        lights: SS_CANdleLight
    ):
        super().__init__()

        self.driver = driver
        self.operator = operator
        self.feeder = feeder
        self.turret = turret
        self.shooter = shooter
        self.camera = camera
        self.swerve = swerve
        self.intake = intake
        self.lights = lights

        self.shoot_balls_sequence = CMD_Shoot.CMDShoot(self.shooter, self.feeder)


        self.configure_driver_inputs()
        self.configure_operator_inputs()

    # ---------------------------------------------------------
    # DRIVER CONTROLS
    # ---------------------------------------------------------
    def configure_driver_inputs(self):

        # Button A
        self.driver.a().whileTrue(self.run_intake_command())

        # Button B
        self.driver.b().whileTrue(self.run_outtake_command())

        # Button Y
        self.driver.y().whileTrue(self.run_vision_align_command())

        # Button X
        self.driver.x().whileTrue(self.run_shooter_spin_command())

        # Left Bumper
        self.driver.leftBumper().whileTrue(self.swerve.field_oriented_command())

        # Left Trigger
        self.driver.leftTrigger(threshold=0.2).whileTrue(self.swerve.precision_mode_command())

        # Right Bumper
        self.driver.rightBumper().whileTrue(self.swerve.robot_oriented_command())

        # Right Trigger
        self.driver.rightTrigger(threshold=0.2).whileTrue(self.swerve.boost_mode_command())
        
        # D-Pad Up
        self.driver.povUp().whileTrue(self.swerve.slow_mode_command())

        # D-Pad Down
        self.driver.povDown().whileTrue(self.swerve.normal_mode_command())

        # D-Pad Left
        self.driver.povLeft().whileTrue(self.swerve.fast_mode_command())

        # D-Pad Right
        self.driver.povRight().whileTrue(self.swerve.turbo_mode_command())
    


    # ---------------------------------------------------------
    # OPERATOR CONTROLS
    # ---------------------------------------------------------
    def configure_operator_inputs(self):

        # Button A
        self.operator.a().whileTrue(self.run_intake_command())

        # Button B
        self.operator.b().whileTrue(self.run_outtake_command())

        # Button Y
        self.operator.y().whileTrue(self.run_vision_align_command())

        # Button X
        self.operator.x().whileTrue(self.run_shooter_spin_command())

        # Left Bumper
        self.operator.leftBumper().whileTrue(self.swerve.field_oriented_command())

        # Left Trigger
        self.operator.leftTrigger(threshold=0.2).whileTrue(self.swerve.precision_mode_command())

        # Right Bumper
        self.operator.rightBumper().whileTrue(self.swerve.robot_oriented_command())

        # Right Trigger
        self.operator.rightTrigger(threshold=0.2).whileTrue(self.swerve.boost_mode_command())
        
        # D-Pad Up
        self.operator.povUp().whileTrue(self.swerve.slow_mode_command())

        # D-Pad Down
        self.operator.povDown().whileTrue(self.swerve.normal_mode_command())

        # D-Pad Left
        self.operator.povLeft().whileTrue(self.swerve.fast_mode_command())

        # D-Pad Right
        self.operator.povRight().whileTrue(self.swerve.turbo_mode_command())

