import wpilib
import commands2
from commands2 import cmd
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterKraken import SS_ShooterKraken
from subsystems.SS_FeederKraken import SS_FeederKraken
from subsystems.SS_IntakeKraken import SS_IntakeKraken
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.CMD_ComboShoot import CMD_ComboShoot
from commands.SEQ_sequences import SEQ_Shoot, SEQ_DeployIntake


class RobotContainer:
    def __init__(self) -> None:
        self.gamepad = CommandXboxController(0)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.ss_shooter = SS_ShooterKraken(3)
        self.ss_feeder = SS_FeederKraken(1)
        self.ss_intake = SS_IntakeKraken(4)
        self.ss_candle_light_rear = SS_CANdleLight(2)
        self.ss_candle_light_front = SS_CANdleLight(5)
        self.ss_swerve_drive = SS_SwerveDrive(self.gamepad)
        self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)

        self.cmd_combo_shoot = CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.gamepad)
        NamedCommands.registerCommand("Combo Shoot", self.cmd_combo_shoot)
        self.seq_shoot = SEQ_Shoot(self.ss_shooter, self.ss_feeder)
        NamedCommands.registerCommand("SEQ Shoot", self.seq_shoot)

        self.auto_chooser = AutoBuilder.buildAutoChooser("None") # must be defined after SS's and all registered commands

        self.configure_gamepad_bindings()
        self.configure_swerve_bindings()

    def configure_gamepad_bindings(self):
        self.gamepad.rightBumper().whileTrue(cmd.startEnd(self.ss_shooter.run_velocity_at_setpoint, 
                                                          self.ss_shooter.stop_motor, self.ss_shooter))
        self.gamepad.leftBumper().whileTrue(cmd.startEnd(self.ss_feeder.run_velocity_at_setpoint,
                                                         self.ss_feeder.stop_motor, self.ss_feeder))
        self.gamepad.a().onTrue(commands2.cmd.startEnd(self.ss_intake.run_intake_in, 
                                                       self.ss_intake.stop_motor, self.ss_intake))
        self.gamepad.b().whileTrue(commands2.cmd.startEnd(self.ss_intake.run_intake_out, 
                                                          self.ss_intake.stop_motor, self.ss_intake))
        self.gamepad.x().onFalse(SEQ_Shoot(self.ss_shooter, self.ss_feeder))
        ...

    def configure_swerve_bindings(self) -> None:
        self.gamepad.x().onTrue(cmd.runOnce(self.ss_swerve_drive.drive_mode_padlocked))
        self.gamepad.x().onFalse(cmd.runOnce(self.ss_swerve_drive.drive_mode_field_centered))
        # self.gamepad.pov(0).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(1, 0), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
        # self.gamepad.pov(180).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(-1, 0), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
        # self.gamepad.pov(90).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(0, 1), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
        # self.gamepad.pov(270).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.pov_move(0, -1), lambda: self.ss_swerve_drive.pov_move(0, 0)) )
        (self.gamepad.back() & self.gamepad.b()).whileTrue(cmd.runOnce(self.ss_swerve_drive.brake))
        (self.gamepad.back() & self.gamepad.start()).onTrue(cmd.runOnce(self.ss_swerve_drive.reset_field_oriented_perspective))

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
        self.container.ss_swerve_drive.drive_mode_field_centered()

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