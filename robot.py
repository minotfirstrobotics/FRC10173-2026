import commands2
import wpilib
from wpilib import SmartDashboard, Timer
from phoenix6 import HootAutoReplay
# from pathplannerlib.auto import AutoBuilder
from commands2.button import CommandXboxController
from subsystems import SS_UptakeTalon
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterSpark import SS_ShooterSpark
from subsystems.SS_TurretTalon_Trapezoidal import SS_TurretTalon
from subsystems.SS_UptakeTalon import SS_UptakeTalon
from subsystems.SS_IntakeSpark import SS_IntakeSpark
from subsystems.SS_CameraPose import SS_CameraPose


class RobotContainer:
    def __init__(self) -> None:
        self.joystick = CommandXboxController(0)
        self.ss_swerve_drive = SS_SwerveDrive(self.joystick)
        self.ss_shooter_spark = SS_ShooterSpark(self.joystick)
        self.ss_turret_talon = SS_TurretTalon(self.joystick)
        self.ss_uptake_talon = SS_UptakeTalon(self.joystick)
        self.ss_intake_spark = SS_IntakeSpark(self.joystick)
        # self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)
        # self.auto_chooser = AutoBuilder.buildAutoChooser("Autonomous Mode")
        # SmartDashboard.putData("Default Autonomous", self.auto_chooser)


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
        # self._time_and_joystick_replay = (HootAutoReplay().with_timestamp_replay().with_joystick_replay() )

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
        commands2.CommandScheduler.getInstance().run()
        # self._time_and_joystick_replay.update() # using HootAutoReplay to log and replay timestamp and joystick data
 
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime())
        ds_match_time = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time (DS)", ds_match_time)
        # fallback when DS time unavailable
        if ds_match_time < 0:
            SmartDashboard.putNumber("Match Time", self.localMatchTimer.get())
        else:
            SmartDashboard.putNumber("Match Time", ds_match_time)


    def teleopInit(self) -> None:
        """This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        if self.autonomousCommand: self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        # self.autonomousCommand = self.container.auto_chooser.getSelected()
        # if self.autonomousCommand: self.autonomousCommand.schedule()

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