import constants
import typing
import commands2
import wpilib
from wpilib import SmartDashboard, Timer
from phoenix6 import HootAutoReplay
# from pathplannerlib.auto import AutoBuilder
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive


class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0) # must be before swerve drive subsystem
        self.initialize_subsystems()
        self.controller_bindings() # must be after subsystems are initialized to bind buttons to subsystem commands
        # self.auto_chooser = AutoBuilder.buildAutoChooser(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"])
        # SmartDashboard.putData(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"], self.auto_chooser)

    def initialize_subsystems(self) -> None:
        self.ss_swerve_drive = SS_SwerveDrive(self.joystick)

    def controller_bindings(self) -> None:
        # joystick bindings for movement are contained in the SS_SwerveDrive class
        self.joystick.a().onTrue(commands2.cmd.runOnce(self.ss_swerve_drive.heading_is_auto_controlled))
        self.joystick.a().onFalse(commands2.cmd.runOnce(self.ss_swerve_drive.heading_is_driver_controlled))
        self.joystick.pov(0).whileTrue(self.ss_swerve_drive.pov_move(1, 0))
        self.joystick.pov(180).whileTrue(self.ss_swerve_drive.pov_move(-1, 0))
        self.joystick.pov(90).whileTrue(self.ss_swerve_drive.pov_move(0, 1))
        self.joystick.pov(270).whileTrue(self.ss_swerve_drive.pov_move(0, -1))


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()
    
    #     self._time_and_joystick_replay = (
    #     HootAutoReplay()
    #     .with_timestamp_replay()
    #     .with_joystick_replay()
    # )

    def robotPeriodic(self) -> None: # Called every 20 ms
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # TODO commands2.CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

        # self._time_and_joystick_replay.update() # using HootAutoReplay to log and replay timestamp and joystick data

        """Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        commands, running already-scheduled commands, removing finished or interrupted commands,
        and running subsystem periodic() methods.  This must be called from the robot's periodic
        block in order for anything in the Command-based framework to work."""
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass

    def autonomousInit(self) -> None:
        pass
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        # self.autonomousCommand = self.container.auto_chooser.getSelected()
        # if self.autonomousCommand: self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass

    def teleopInit(self) -> None:
        """This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. """
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass

    def testInit(self) -> None:
        """Cancels all running commands at the start of test mode"""
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)