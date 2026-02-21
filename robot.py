import constants
import typing
import commands2
import wpilib
from wpilib import SmartDashboard, Timer
# from pathplannerlib.auto import AutoBuilder
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive


class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0)
        self.initialize_swerve_drive()
        self.initialize_subsystems()
        self.controller_bindings()


    def initialize_swerve_drive(self) -> None:
        self.ss_swerve_drive = SS_SwerveDrive(self.joystick)
#        self._auto_chooser = AutoBuilder.buildAutoChooser(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"])
        # SmartDashboard.putData(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"], self._auto_chooser)


    def getAutonomousCommand(self) -> commands2.Command:
        return self._auto_chooser.getSelected()


    def initialize_subsystems(self) -> None:
        pass


    def controller_bindings(self) -> None:
        # joystick bindings for movement are contained in the SS_SwerveDrive class
        self.joystick.pov(0).whileTrue(self.ss_swerve_drive.pov_move(1, 0))
        self.joystick.pov(180).whileTrue(self.ss_swerve_drive.pov_move(-1, 0))
        self.joystick.pov(270).whileTrue(self.ss_swerve_drive.pov_move(0, -1))
        self.joystick.pov(90).whileTrue(self.ss_swerve_drive.pov_move(0, 1))
        

class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None: # Should this be __init__?
        self.container = RobotContainer()

    def robotPeriodic(self) -> None: # Called every 20 ms
        # TODO commands2.CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime())


    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()


if __name__ == "__main__":
    wpilib.run(MyRobot)