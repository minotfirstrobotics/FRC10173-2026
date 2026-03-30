import wpilib
import commands2
from commands2 import cmd
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from generated.tuner_constants_2026_GF import TunerConstants
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_Kraken import SS_Kraken
from examples.SS_ShooterKraken import SS_ShooterKraken
from examples.SS_FeederKraken import SS_FeederKraken
from examples.SS_IntakeKraken import SS_IntakeKraken
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.CMD_ComboShoot import CMD_ComboShoot
from commands.SEQ_sequences import SEQ_Shoot, SEQ_DeployIntake


class RobotContainer:
    def __init__(self) -> None:
        self.gamepad = None or CommandXboxController(0)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.canbus = TunerConstants.canbus
        # self.ss_shooter = None or SS_ShooterKraken(3, self.canbus)
        self.ss_shooter = None or SS_Kraken(3, self.canbus, "Shooter", max_rps=100, velocity_setpoint=40, kp=0.01, ki=0.0, kd=0.0, kv=0.01, ks=0.0)
        # self.ss_feeder = None or SS_FeederKraken(1, self.canbus)
        self.ss_feeder = None or SS_Kraken(1, self.canbus, "Feeder", inverted=True, percent_power_setpoint=0.62)
        # self.ss_intake = None #or SS_IntakeKraken(4, self.canbus)
        self.ss_intake = None #SS_Kraken(4, self.canbus, "Intake", inverted=True, max_rps=120, percent_power_setpoint=0.62)
        # self.ss_candle_light_rear = None or SS_CANdleLight(2, self.canbus)
        # self.ss_candle_light_front = None or SS_CANdleLight(5, self.canbus)
        self.ss_swerve_drive = None or SS_SwerveDrive(self.gamepad)
        # self.ss_camera_pose = None or SS_CameraPose(self.ss_swerve_drive)

        # ------------------ Complex Commands and Auto Builder ------------------
        self.cmd_combo_shoot = CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.gamepad)
        NamedCommands.registerCommand("Combo Shoot", self.cmd_combo_shoot)
        self.seq_shoot = SEQ_Shoot(self.ss_shooter, self.ss_feeder)
        NamedCommands.registerCommand("SEQ Shoot", self.seq_shoot)
        self.auto_chooser = AutoBuilder.buildAutoChooser("None") # must be defined after SS's and all registered commands
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

        if self.gamepad:
             self.configure_gamepad_bindings()

    def configure_gamepad_bindings(self):

        ## Operations bindings
        if self.ss_shooter:
            self.gamepad.rightBumper().whileTrue(cmd.startEnd(
                self.ss_shooter.run_velocity_at_setpoint, self.ss_shooter.stop_motor, self.ss_shooter))
        if self.ss_feeder:
            self.gamepad.leftBumper().whileTrue(cmd.startEnd(
                self.ss_feeder.run_velocity_at_setpoint, self.ss_feeder.stop_motor, self.ss_feeder))
        if self.ss_intake:
            self.gamepad.leftTrigger(threshold=.2).whileTrue(commands2.cmd.startEnd(
                self.ss_intake.run_intake_in, self.ss_intake.stop_motor, self.ss_intake))
            self.gamepad.rightTrigger(threshold=.2).whileTrue(commands2.cmd.startEnd(
                self.ss_intake.run_intake_out, self.ss_intake.stop_motor, self.ss_intake))
        if self.ss_shooter and self.ss_feeder:
            self.gamepad.x().onFalse(SEQ_Shoot(self.ss_shooter, self.ss_feeder))

        if self.ss_swerve_drive:
            ## Set starting drive mode to field-centered, and allow toggling to padlocked with the A button
            self.ss_swerve_drive.drive_mode_field_centered()
            # self.ss_swerve_drive.drive_mode_padlocked()

            self.gamepad.a().onTrue(cmd.runOnce(self.ss_swerve_drive.drive_mode_padlocked))
            self.gamepad.a().onFalse(cmd.runOnce(self.ss_swerve_drive.drive_mode_field_centered))
            # self.gamepad.a().and_(self.gamepad.back()).onFalse(cmd.runOnce(self.ss_swerve_drive.change_target))
            # self.gamepad.pov(0).whileTrue(self.ss_swerve_drive.pov_move(1, 0))
            # self.gamepad.pov(180).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(-1, 0), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
            # self.gamepad.pov(90).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(0, 1), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
            # self.gamepad.pov(270).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(0, -1), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
            self.gamepad.back().and_(self.gamepad.b()).whileTrue(cmd.runOnce(self.ss_swerve_drive.brake))
            self.gamepad.back().and_(self.gamepad.start()).onTrue(cmd.runOnce(self.ss_swerve_drive.reset_field_oriented_perspective))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.auto_chooser.getSelected()


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.

        Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        autonomous chooser on the dashboard.
        """
        self.autonomousCommand = None
        self.container = RobotContainer()
        self.localMatchTimer = Timer()
        # self._time_and_driver_replay = (HootAutoReplay().with_timestamp_replay().with_driver_replay() )

    def robotPeriodic(self) -> None:
        """
        This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating.

        Starts by running the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        commands, running already-scheduled commands, removing finished or interrupted commands,
        and running subsystem periodic() methods.  This must be called from the robot's periodic
        block in order for anything in the Command-based framework to work.
        """
        # self._time_and_driver_replay.update() # using HootAutoReplay to log and replay timestamp and driver data
        match_time_from_driver_station = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time", self.localMatchTimer.get() if match_time_from_driver_station < 0 else match_time_from_driver_station)


    def teleopInit(self) -> None:
        """
        This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. 
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        """
        This function is run once when the robot enters autonomous mode.
        Gets the selected autonomous command from the chooser and schedules it.
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        self.autonomousCommand = self.container.getAutonomousCommand()
        # self.autonomousCommand = SEQ_DeployIntake(self.container.ss_swerve_drive)
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass


    def testInit(self) -> None:
        """Cancels all running commands at the start of test mode"""
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)