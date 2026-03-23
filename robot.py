import commands2
import wpilib
from wpilib import SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands
from commands2.button import CommandXboxController
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_ShooterKraken import SS_ShooterKraken
from examples.SS_FeederTalon_Power import SS_FeederTalon_Power
from examples.SS_IntakeSIMM import SS_IntakeSIMM
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.CMD_ComboShoot import CMD_ComboShoot
from commands.SEQ_sequences import SEQ_Shoot, SEQ_DeployIntake


class RobotContainer:
    def __init__(self) -> None:
        self.gamepad = CommandXboxController(0)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.ss_shooter = SS_ShooterKraken(3)
        # self.ss_feeder = SS_FeederTalon_Power(0)
        # self.ss_intake = SS_IntakeSIMM(4)
        # self.ss_candle_light_rear = SS_CANdleLight(2)
        self.ss_candle_light_front = SS_CANdleLight(5)
        self.ss_swerve_drive = SS_SwerveDrive(self.gamepad)
        # self.ss_camera_pose = SS_CameraPose(self.ss_swerve_drive)

        # Build auto chooser from PathPlanner auto.auto files in deploy folder
        self.auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Autonomous Routine", self.auto_chooser)

        self.configure_gamepad_bindings()
        self.configure_swerve_bindings()

    def configure_gamepad_bindings(self):
        self.gamepad.rightBumper().whileTrue(self.ss_shooter.run_setpoint_velocity_command())
        # self.gamepad.leftBumper().whileTrue(self.ss_feeder.run_feeder_command())
        # self.gamepad.a().whileTrue(self.ss_intake.run_intake_command())

    def configure_swerve_bindings(self) -> None:
        self.gamepad.x().onTrue(self.ss_swerve_drive.heading_is_driver_padlocked_command())
        self.gamepad.x().onFalse(self.ss_swerve_drive.heading_is_driver_controlled_command())
        # self.gamepad.pov(0).whileTrue(self.pov_move_command(1, 0))
        # self.gamepad.pov(180).whileTrue(self.pov_move_command(-1, 0))
        # self.gamepad.pov(90).whileTrue(self.pov_move_command(0, 1))
        # self.gamepad.pov(270).whileTrue(self.pov_move_command(0, -1))
        (self.gamepad.back() & self.gamepad.b()).whileTrue(self.ss_swerve_drive.brake_command())
        (self.gamepad.back() & self.gamepad.start()).onTrue(self.ss_swerve_drive.reset_field_oriented_perspective_command())

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
        commands2.CommandScheduler.getInstance().run()
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

        # Get the selection from the chooser. Some chooser implementations
        # may return a callable that constructs a command, or return an
        # object that already is a command. Be defensive and handle both.
        self.autonomousCommand = self.container.getAutonomousCommand()

        # If the chooser returned a factory (callable), call it to get the actual command instance.
        if callable(self.autonomousCommand):
            try:
                self.autonomousCommand = self.autonomousCommand()
            except Exception as e:
                # If the factory raises, log and clear autonomousCommand.
                print(f"Error creating autonomous command from chooser: {e}")
                self.autonomousCommand = None

        # If the selected object has a schedule() method, schedule it.
        if self.autonomousCommand is not None and hasattr(self.autonomousCommand, "schedule"):
            try:
                self.autonomousCommand.schedule()
            except Exception as e:
                print(f"Failed to schedule autonomous command: {e}")
                self.autonomousCommand = None
        else:
            # Helpful debug information when something unexpected is returned
            if self.autonomousCommand is not None:
                print(f"Autonomous selection is not scheduleable: {type(self.autonomousCommand)!r} -> {self.autonomousCommand!r}")

        # Klingbeil's old manual autonomous command scheduling (before PathPlanner auto.auto files and AutoBuilder)
        # auto = SEQ_DeployIntake(self.container.ss_swerve_drive, self.container.ss_shooter_spark, self.container.ss_feeder_talon, self.container.ss_intake_spark)
        # auto.schedule()

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